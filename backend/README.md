# RAG Chatbot Backend

This directory contains the backend service for the Integrated RAG Chatbot.

## 1. Prerequisites

- Python 3.11+
- `pip` with `venv` (or a similar virtual environment tool)
- Access to the command line / terminal.
- API keys and URLs for required services (Qdrant, Neon, Gemini).

## 2. Environment Setup

1.  **Navigate to this directory**:
    ```bash
    cd backend
    ```

2.  **Create a `.env` file**. This file will store your secret keys and service URLs. You can copy the example file:
    ```bash
    cp .env.example .env
    ```

3.  **Populate the `.env` file** with the following key-value pairs. Replace the placeholder values with your actual credentials.

    ```env
    # Qdrant Cloud Credentials
    QDRANT_URL="https://your-qdrant-cluster-url.qdrant.tech:6333"
    QDRANT_API_KEY="your-qdrant-api-key"

    # Neon Serverless Postgres
    NEON_DATABASE_URL="postgresql://user:password@host:port/dbname"

    # Gemini API Credentials
    GEMINI_API_KEY="your-gemini-api-key"
    ```

## 3. Installation

1.  **Create and activate a virtual environment**:
    ```bash
    # Create the environment
    python -m venv .venv

    # Activate on macOS/Linux
    source .venv/bin/activate

    # Activate on Windows
    # .\.venv\Scripts\activate
    ```

2.  **Install dependencies from `pyproject.toml`**:
    ```bash
    pip install -e .
    ```
    (Note: You might need to install `uvicorn` separately: `pip install uvicorn[standard]`)

## 4. Running the Service

Once the setup is complete, you can run the FastAPI web server.

```bash
# The 'uvicorn' command starts the web server.
# The --reload flag enables hot-reloading for development.
uvicorn src.api.main:app --reload
```

The service will be available at `http://127.0.0.1:8000`.

## 5. API Endpoints

You can interact with the API using any HTTP client or by visiting `http://127.0.0.1:8000/docs` in your browser for the Swagger UI.

### POST /query

Query the chatbot.

**Request Body:**
```json
{
  "query": "What is a URDF?",
  "user_id": "session_xyz789",
  "selected_text": "A link element describes one rigid body..."
}
```

### DELETE /history/{user_id}

Delete the conversation history for a user.

## 6. Running Tests (Not Implemented)

To run tests, you would typically use `pytest`:
```bash
pytest
```
(Note: Tests have not been implemented yet as part of this task list.)
