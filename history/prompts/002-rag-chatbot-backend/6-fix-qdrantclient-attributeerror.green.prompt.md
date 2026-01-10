---
id: 6
title: Fix QdrantClient AttributeError
stage: green
date: 2026-01-01
surface: agent
model: gemini-pro
feature: 002-rag-chatbot-backend
branch: main
user: unknown
command: unknown
labels: ["bug-fix", "library-update", "qdrant"]
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

now it's giving this error, fix this `2026-01-01 18:42:18,174 - src.api.main - ERROR - An unexpected error occurred during query processing: 'QdrantClient' object has no attribute 'search'
INFO:     127.0.0.1:51722 - "POST /query HTTP/1.1" 503 Service Unavailable`

## Response snapshot

I have updated `backend/src/services/rag_service.py` to use the `query_points` method instead of the deprecated `search` method. This should resolve the error you were seeing.

Please try running your application again.

## Outcome

- ✅ Impact: High. The change fixes a critical error caused by a breaking change in a dependency library (`qdrant-client`).
- 🧪 Tests: No tests were run. The fix was based on documentation for the updated library.
- 📁 Files: Modified `backend/src/services/rag_service.py` to use the new `query_points` method.
- 🔁 Next prompts: The user can continue testing their application.
- 🧠 Reflection: The error `'QdrantClient' object has no attribute 'search'` was a strong indicator of a breaking change in the `qdrant-client` library, as the installed version (`^1.7.0`) could have pulled in a newer minor version. The changelog confirmed the deprecation and removal of the `search` method. The fix involved replacing it with the new `query_points` method.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
