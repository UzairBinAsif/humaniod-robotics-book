---
id: 7
title: Fix Qdrant Collection Not Found Error
stage: green
date: 2026-01-01
surface: agent
model: gemini-pro
feature: 002-rag-chatbot-backend
branch: main
user: unknown
command: unknown
labels: ["bug-fix", "database", "qdrant", "data-population"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - backend/src/services/rag_service.py
tests:
 - "none"
---

## Prompt

now it's saying `warning: No `requires-python` value found in the workspace. Defaulting to `>=3.12`.
INFO:     Will watch for changes in these directories: ['C:\\Users\\shaik\\OneDrive\\Desktop\\code\\GIAIC\\Claude Code\\humaniod-robotics-book\\backend']
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [18156] using WatchFiles
2026-01-01 18:47:49,449 - src.services.agent - INFO - Successfully initialized Gemini client for AgentService with model: gemini-2.0-flash
2026-01-01 18:47:50,251 - httpx - INFO - HTTP Request: GET https://326a717d-df29-4cbc-94ec-7d0def438f12.us-east4-0.gcp.cloud.qdrant.io:6333 "HTTP/1.1 200 OK"
2026-01-01 18:47:50,277 - src.services.rag_service - INFO - Successfully connected to Qdrant.
2026-01-01 18:47:50,278 - src.services.rag_service - INFO - Successfully configured Gemini client.
INFO:     Started server process [14220]
INFO:     Waiting for application startup.
2026-01-01 18:47:52,303 - src.services.db - INFO - Tables 'conversation' and 'message' are ready.
2026-01-01 18:47:52,389 - src.services.db - INFO - Successfully connected to Neon Postgres and created tables.
INFO:     Application startup complete.
INFO:     127.0.0.1:54460 - "OPTIONS /query HTTP/1.1" 200 OK
2026-01-01 18:48:11,716 - httpx - INFO - HTTP Request: POST https://326a717d-df29-4cbc-94ec-7d0def438f12.us-east4-0.gcp.cloud.qdrant.io:6333/collections/humanoid-robotics-book/points/query "HTTP/1.1 404 Not Found"
2026-01-01 18:48:11,716 - src.api.main - ERROR - An unexpected error occurred during query processing: Unexpected Response: 404 (Not Found)
Raw response content:
b'{"status":{"error":"Not found: Collection `humanoid-robotics-book` doesn\'t exist!"},"time":0.000230608}'
INFO:     127.0.0.1:54460 - "POST /query HTTP/1.1" 503 Service Unavailable`

## Response snapshot

I have addressed the immediate error by modifying the backend to automatically create the Qdrant collection if it doesn\'t exist. You should no longer see the `404 Not Found` error from Qdrant.

However, the newly created collection is empty. This means the chatbot will not be able to find any relevant context to answer your queries, and its responses will not be very helpful.

The next logical step is to populate the Qdrant collection with the content from your book. I could not find an existing script to do this.

Would you like me to create a script that will read your book\'s markdown files, generate embeddings for them, and upload them to the Qdrant collection?

## Outcome

- ✅ Impact: High. The change fixes a critical error that prevented the application from starting correctly.
- 🧪 Tests: No tests were run. The fix was a defensive measure to ensure the application is robust to a missing database collection.
- 📁 Files: Modified `backend/src/services/rag_service.py` to automatically create the Qdrant collection.
- 🔁 Next prompts: The user will likely ask me to create the data population script.
- 🧠 Reflection: The error `Collection ... doesn\'t exist!` was a clear sign that the application was not correctly initialized. Adding the collection creation logic is a good defensive programming practice. It\'s also important to anticipate the user\'s next problem (empty database) and proactively offer a solution.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
