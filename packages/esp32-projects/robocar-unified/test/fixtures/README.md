# Gemini ER 1.6 response fixtures

These JSON files are captured from live
`generativelanguage.googleapis.com/v1beta/models/gemini-robotics-er-1.6:generateContent`
calls and used by `test_gemini_parse.c` to lock down the shape assumptions of
`gemini_parse_function_call()`.

## Current status

The committed files are **synthetic** fixtures that match the schema documented
in `gemini_parse.c`. They prove the parser reads the expected path, but they
do not guarantee the live API still emits the same shape. Closing PRP gate
**G1** requires replacing each file with a recording from a real API call —
see **Capturing a real fixture** below.

| File | Scenario | Synthetic | Captured |
|------|----------|:---:|:---:|
| `er_1_6_drive.json`   | straight-line drive    | ✅ | ☐ |
| `er_1_6_track.json`   | track a bounding box   | ✅ | ☐ |
| `er_1_6_rotate.json`  | rotate in place        | ✅ | ☐ |
| `er_1_6_stop.json`    | stop                    | ✅ | ☐ |
| `er_1_6_corrupt.json` | malformed response → fail-safe contract | n/a | n/a |

## Capturing a real fixture

You need a `GEMINI_API_KEY` with access to Robotics-ER.

```bash
# 1. Grab a representative JPEG (any scene works; a cup on a table is classic).
curl -L -o /tmp/scene.jpg https://picsum.photos/seed/robot/640/480

# 2. Base64 the image.
B64=$(base64 -i /tmp/scene.jpg | tr -d '\n')

# 3. Build the request body — match the shape in gemini_backend.c's
#    build_request_json() so you exercise the same function declarations
#    and toolConfig the firmware sends. The stub below embeds the minimum.
cat > /tmp/req.json <<EOF
{
  "contents": [{
    "role": "user",
    "parts": [
      {"inlineData": {"mimeType": "image/jpeg", "data": "${B64}"}},
      {"text": "Choose one action: drive, track, rotate, or stop."}
    ]
  }],
  "tools": [{
    "functionDeclarations": [
      {"name": "drive",  "description": "Drive on heading.",
       "parameters": {"type": "OBJECT", "properties": {
         "heading_deg": {"type": "INTEGER"},
         "distance_cm": {"type": "INTEGER"},
         "speed_pct":   {"type": "INTEGER"}},
         "required": ["heading_deg","distance_cm","speed_pct"]}},
      {"name": "track",  "description": "Visually servo toward bbox.",
       "parameters": {"type": "OBJECT", "properties": {
         "box_2d":        {"type": "ARRAY", "items": {"type": "INTEGER"}},
         "max_speed_pct": {"type": "INTEGER"}},
         "required": ["box_2d","max_speed_pct"]}},
      {"name": "rotate", "description": "Rotate in place.",
       "parameters": {"type": "OBJECT", "properties": {
         "angle_deg": {"type": "INTEGER"}},
         "required": ["angle_deg"]}},
      {"name": "stop",   "description": "Halt.",
       "parameters": {"type": "OBJECT", "properties": {}, "required": []}}
    ]
  }],
  "toolConfig": {"functionCallingConfig": {"mode": "ANY"}},
  "generationConfig": {"thinkingConfig": {"thinkingBudget": 0}}
}
EOF

# 4. Call the API, save the raw response.
curl -s \
  -H "Content-Type: application/json" \
  -H "x-goog-api-key: ${GEMINI_API_KEY}" \
  -X POST \
  -d @/tmp/req.json \
  'https://generativelanguage.googleapis.com/v1beta/models/gemini-robotics-er-1.6:generateContent' \
  | jq . > er_1_6_<scenario>.json

# 5. Run the host tests — they should still pass against the live capture.
cd .. && cmake --build build && ctest --test-dir build --output-on-failure
```

Replace the synthetic fixtures one scenario at a time; run the parse tests
after each swap to catch schema drift. If a field rename breaks parsing,
update `gemini_parse.c` to match what the live API emits and document the
change in `ADR-016`.
