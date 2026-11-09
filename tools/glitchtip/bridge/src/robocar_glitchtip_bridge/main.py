"""Subscribe to robot events over MQTT and forward each one to GlitchTip.

Configuration (environment):
  MQTT_URL                  mqtt://[user:pass@]host[:port]   (default mqtt://localhost:1883)
  MQTT_TOPIC                topic to subscribe               (default robocar/events)
  GLITCHTIP_DSN_ROBOCAR     DSN of the `robocar` project     (source=firmware)
  GLITCHTIP_DSN_ROBOCAR_AI  DSN of the `robocar-ai` project  (source=planner)
  ELF_DIR                   directory of <build-sha>.elf     (default ./elf)
  ELF_RELEASE_URL           release ELF URL, {version} filled in; empty disables
                            (default: this repo's robocar-unified-v{version} release)

The MQTT session is persistent (clean_session=False, fixed client id), so the
broker queues QoS 1 events while the bridge is down.
"""

from __future__ import annotations

import functools
import logging
import os
import sys
import time
from pathlib import Path
from urllib.parse import urlparse

import paho.mqtt.client as mqtt
import sentry_sdk

from .event import InvalidEvent, build_sha, parse_payload, route, to_sentry_event
from .release_elf import DEFAULT_URL_TEMPLATE, fetch_release_elf, release_version
from .symbolicate import ElfStore, lookup

log = logging.getLogger("robocar-glitchtip-bridge")

DSN_ENV = {"robocar": "GLITCHTIP_DSN_ROBOCAR", "robocar-ai": "GLITCHTIP_DSN_ROBOCAR_AI"}
CLIENT_ID = "robocar-glitchtip-bridge"


def make_clients() -> dict[str, sentry_sdk.Client]:
    """One Client per GlitchTip project, none of them installed globally.

    Events are sent with `client.capture_event`, not `Scope.capture_event`:
    `Scope.get_client` is a classmethod that reads the *current* scope, so a
    Scope built with its own client still sends through the global one (a
    no-op client here).
    """
    clients = {}
    for project, env in DSN_ENV.items():
        dsn = os.environ.get(env)
        if not dsn:
            log.warning("%s unset: events for project %s will be dropped", env, project)
            continue
        clients[project] = sentry_sdk.Client(
            dsn=dsn,
            default_integrations=False,
            send_default_pii=False,
            server_name=CLIENT_ID,  # not the bridge host's name
        )
    return clients


def handle(
    payload: bytes, clients: dict[str, sentry_sdk.Client], elves: ElfStore
) -> str | None:
    """Forward one payload; returns the event id sent, or None if dropped."""
    try:
        event = parse_payload(payload)
    except InvalidEvent as exc:
        log.warning("dropped malformed event: %s", exc)
        return None
    project = route(event)
    client = clients.get(project)
    if client is None:
        log.warning("dropped %s: no DSN for project %s", event["event_id"], project)
        return None
    symbolicate = None
    sha = build_sha(event)
    if "exception" in event and sha:
        table = elves.table_for(sha, release_version(event))
        if table is None:
            log.info("no %s/%s.elf: frames stay unsymbolicated", elves.directory, sha)
        else:
            symbolicate = functools.partial(lookup, table)
    sentry_event = to_sentry_event(
        event, received_at=time.time(), symbolicate=symbolicate
    )
    sent = client.capture_event(sentry_event, scope=sentry_sdk.Scope())
    client.flush(timeout=5)
    if sent is None:
        log.warning("sentry-sdk dropped %s before sending", event["event_id"])
    else:
        log.info("forwarded %s to %s: %s", sent, project, event["message"][:80])
    return sent


def _fetch(sha: str, version: str | None, *, directory: Path, template: str):
    return fetch_release_elf(sha, version, directory, template)


def main() -> None:
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    url = urlparse(os.environ.get("MQTT_URL", "mqtt://localhost:1883"))
    topic = os.environ.get("MQTT_TOPIC", "robocar/events")
    elf_dir = Path(os.environ.get("ELF_DIR", "elf"))
    template = os.environ.get("ELF_RELEASE_URL", DEFAULT_URL_TEMPLATE)
    elves = ElfStore(
        elf_dir,
        fetch=functools.partial(_fetch, directory=elf_dir, template=template),
    )
    clients = make_clients()
    if not clients:
        sys.exit("no GLITCHTIP_DSN_* set; run `just glitchtip::bootstrap` first")

    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, client_id=CLIENT_ID, clean_session=False
    )
    if url.username:
        client.username_pw_set(url.username, url.password)

    def on_connect(client, _userdata, _flags, reason_code, _properties):
        if reason_code.is_failure:
            log.error("MQTT connect failed: %s", reason_code)
            return
        log.info("connected to %s, subscribing to %s", url.hostname, topic)
        client.subscribe(topic, qos=1)

    def on_message(_client, _userdata, msg):
        try:
            handle(msg.payload, clients, elves)
        except (
            Exception
        ):  # keep the bridge alive; the event is logged, not lost silently
            log.exception("failed to forward a message on %s", msg.topic)

    host, port = url.hostname or "localhost", url.port or 1883

    # A refused or unreachable broker never reaches on_connect, so without
    # this the bridge retries in silence and looks like it is running.
    def on_connect_fail(_client, _userdata):
        log.warning(
            "cannot reach MQTT broker %s:%s; retrying (check MQTT_URL)", host, port
        )

    def on_disconnect(_client, _userdata, _flags, reason_code, _properties):
        log.warning(
            "disconnected from %s:%s (%s); reconnecting", host, port, reason_code
        )

    client.on_connect = on_connect
    client.on_connect_fail = on_connect_fail
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    log.info(
        "connecting to %s:%s%s, topic %s",
        host,
        port,
        f" as {url.username}" if url.username else " (anonymous)",
        topic,
    )
    client.connect_async(host, port)
    client.loop_forever(retry_first_connection=True)


if __name__ == "__main__":
    main()
