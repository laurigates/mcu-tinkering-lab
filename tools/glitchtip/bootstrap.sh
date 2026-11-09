#!/usr/bin/env bash
# Create a GlitchTip admin, an organisation and the robocar projects, then
# write each project's DSN to .env. Idempotent: re-running keeps existing
# projects and keys.
#
# GlitchTip's REST bootstrap endpoints change between versions, so this drives
# the Django ORM through `manage.py shell` in the running web container.
#
# Environment (all optional):
#   GLITCHTIP_ORG             organisation name             (default robocar-lab)
#   GLITCHTIP_PROJECTS        space-separated project names (default "robocar robocar-ai")
#   GLITCHTIP_ADMIN_EMAIL     admin login                   (default admin@example.com)
#   GLITCHTIP_ADMIN_PASSWORD  admin password                (default: generated, saved to .env)
#   GLITCHTIP_DSN_HOST        host:port in host-side DSNs   (default localhost:$GLITCHTIP_PORT)
set -euo pipefail

cd "$(dirname "$0")"
[ -f .env ] || { echo "no .env; run 'just glitchtip::up' first" >&2; exit 1; }
set -a
# shellcheck disable=SC1091
source .env
set +a

ORG="${GLITCHTIP_ORG:-robocar-lab}"
PROJECTS="${GLITCHTIP_PROJECTS:-robocar robocar-ai}"
EMAIL="${GLITCHTIP_ADMIN_EMAIL:-admin@example.com}"
PASSWORD="${GLITCHTIP_ADMIN_PASSWORD:-$(openssl rand -hex 12)}"
DSN_HOST="${GLITCHTIP_DSN_HOST:-localhost:${GLITCHTIP_PORT:-8000}}"
COMPOSE=(docker compose --project-directory . -f compose.yml)

# Values reach Python through the environment, never by splicing into code.
OUT=$("${COMPOSE[@]}" exec -T \
    -e BS_ORG="$ORG" -e BS_PROJECTS="$PROJECTS" -e BS_EMAIL="$EMAIL" -e BS_PASSWORD="$PASSWORD" \
    web ./manage.py shell -c "
import os
from django.apps import apps
from django.contrib.auth import get_user_model
Organization = apps.get_model('organizations_ext', 'Organization')
OrganizationUser = apps.get_model('organizations_ext', 'OrganizationUser')
OrganizationOwner = apps.get_model('organizations_ext', 'OrganizationOwner')
Project = apps.get_model('projects', 'Project')
ProjectKey = apps.get_model('projects', 'ProjectKey')
user, created = get_user_model().objects.get_or_create(email=os.environ['BS_EMAIL'])
if created:
    user.set_password(os.environ['BS_PASSWORD'])
    user.is_staff = user.is_superuser = True
    user.save()
org, _ = Organization.objects.get_or_create(name=os.environ['BS_ORG'])
ou, _ = OrganizationUser.objects.get_or_create(user=user, organization=org, defaults={'role': 3})  # 3 = owner
OrganizationOwner.objects.get_or_create(organization=org, defaults={'organization_user': ou})
print('USER_CREATED', created)
for name in os.environ['BS_PROJECTS'].split():
    proj, _ = Project.objects.get_or_create(name=name, organization=org)
    key = ProjectKey.objects.filter(project=proj).first() or ProjectKey.objects.create(project=proj)
    print('PROJECT', name, key.public_key.hex, proj.id)
")

# Replace (or add) KEY=value in .env.
set_env() {
    grep -v "^$1=" .env > .env.tmp || true
    printf '%s=%s\n' "$1" "$2" >> .env.tmp
    mv .env.tmp .env
}

if grep -q '^USER_CREATED True' <<<"$OUT"; then
    set_env GLITCHTIP_ADMIN_EMAIL "$EMAIL"
    set_env GLITCHTIP_ADMIN_PASSWORD "$PASSWORD"
    echo "created admin $EMAIL (password in .env)"
else
    echo "admin $EMAIL already exists; password unchanged"
fi

found=0
while read -r _ name key id; do
    var="GLITCHTIP_DSN_$(tr 'a-z-' 'A-Z_' <<<"$name")"
    set_env "$var" "http://$key@$DSN_HOST/$id"
    set_env "${var}_COMPOSE" "http://$key@web:8000/$id"
    echo "$var=http://$key@$DSN_HOST/$id"
    found=$((found + 1))
done < <(grep '^PROJECT ' <<<"$OUT")

if [ "$found" -eq 0 ]; then
    echo "no projects reported; manage.py output was:" >&2
    echo "$OUT" >&2
    exit 1
fi
echo "UI: ${GLITCHTIP_DOMAIN:-http://localhost:8000} (org $ORG)"
