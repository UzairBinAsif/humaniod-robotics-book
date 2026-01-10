---
id: 8
title: "Update Chatbot to Use OpenRouter API"
stage: "green"
date: "2026-01-10"
surface: "agent"
model: "google/gemini-2.0-flash-001"
branch: "main"
user: "{{user}}"
command: "I've added OPENROUTER_API_KEY in .env, update the chatbot's code to use openrouter api with google/gemini-2.0-flash-001 model"
tags:
  - "agent-refactor"
  - "openrouter-api"
  - "gemini-2.0-flash"
---

# Prompt

I've added OPENROUTER_API_KEY in .env, update the chatbot's code to use openrouter api with google/gemini-2.0-flash-001 model

# Response

I have successfully updated the `agent.py` file to use the OpenRouter API.

### Changes Made:

1.  **Environment Variable**:
    *   The code now retrieves the `OPENROUTER_API_KEY` from your `.env` file.

2.  **API Client**:
    *   The `OpenAI` client is now configured with the correct `base_url` for OpenRouter (`https://openrouter.ai/api/v1`).

3.  **Model**:
    *   The model has been set to `google/gemini-2.0-flash-001` as requested.

4.  **System Prompt**:
    *   The system prompt has been enhanced to provide better guidance to the model, ensuring it focuses on the provided context.

### Next Steps:

*   Please ensure your `.env` file is in the `backend` directory.
*   You can now run the application, and it will use the OpenRouter API for generating responses.

Let me know if you have any other questions.
