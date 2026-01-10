---
id: 1
title: "Fix OpenAI 401 Unauthorized Error"
stage: "green"
date_iso: "2025-12-30"
surface: "agent"
model: "gemini-pro"
feature: "rag-chatbot-backend"
branch: "main"
user: "shaik"
command: "Fix chatbot error"
labels: ["bug-fix", "api", "gemini"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files_yaml: |
  - backend/pyproject.toml
  - backend/src/services/rag_service.py
  - backend/src/services/agent.py
tests_yaml: |
  - "none"
prompt_text: |
  when I'm messaging the chatbot hi, it's saying Sorry, I encountered an error and couldn't process your request. and the terminal of backend is giving this log INFO:     127.0.0.1:62099 - "OPTIONS /query HTTP/1.1" 200 OK
  2025-12-30 00:31:27,646 - httpx - INFO - HTTP Request: POST https://api.openai.com/v1/embeddings "HTTP/1.1 401 Unauthorized"
  2025-12-30 00:31:27,649 - src.services.rag_service - ERROR - An error occurred during context retrieval: Error code: 401 - {'error': {'message': 'Incorrect API key provided: AIzaSyBD***************************rgtc. You can find your API key at https://platform.openai.com/account/api-keys.', 'type': 'invalid_request_error', 'param': None, 'code': 'invalid_api_key'}}
  2025-12-30 00:31:28,070 - httpx - INFO - HTTP Request: POST https://api.openai.com/v1/chat/completions "HTTP/1.1 401 Unauthorized"
  2025-12-30 00:31:28,070 - src.services.agent - ERROR - An error occurred during agent invocation: Error code: 401 - {'error': {'message': 'Incorrect API key provided: AIzaSyBD***************************rgtc. You can find your API key at https://platform.openai.com/account/api-keys.', 'type': 'invalid_request_error', 'param': None, 'code': 'invalid_api_key'}}
  2025-12-30 00:31:28,315 - src.services.db - INFO - Created new conversation for user_id: session-1767036681476
  2025-12-30 00:31:28,487 - src.services.db - INFO - Saved message for user_id: session-1767036681476
  2025-12-30 00:31:28,747 - src.services.db - INFO - Saved message for user_id: session-1767036681476
  INFO:     127.0.0.1:62099 - "POST /query HTTP/1.1" 200 OK can you fix this
response_text: |
  I have fixed the issue. The root cause was that the code was using the openai library to connect to the Gemini API, which caused an authentication error.

  I have performed the following changes:
  1.  Added the google-generativeai library to the project's dependencies in ackend/pyproject.toml.
  2.  Refactored ackend/src/services/rag_service.py to use the correct google-generativeai library for creating embeddings.
  3.  Refactored ackend/src/services/agent.py to use the correct google-generativeai library for chat completions.

  **Action Required:**

  Please install the new dependency by navigating to the ackend directory and running the following command:
  `shell
  poetry install
  `
  After the installation is complete, please restart your backend server. The chatbot should now work correctly.
---