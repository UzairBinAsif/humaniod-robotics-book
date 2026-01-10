# Quickstart: Integrated RAG Chatbot Backend

This guide provides the steps to set up and run the RAG chatbot backend service.

## 1. Prerequisites

- Python 3.11+
- `pip` with `venv` (or a similar virtual environment tool like `uv`)
- Access to the command line / terminal.
- API keys and URLs for required services (Qdrant, Neon, Gemini).

## 2. Environment Setup

1.  **Navigate to the backend directory**:
    ```bash
    cd backend
    ```

2.  **Create a `.env` file**. This file will store your secret keys and service URLs.

3.  **Populate the `.env` file** with the following key-value pairs. Replace the placeholder values with your actual credentials.

    ```env
    # Qdrant Cloud Credentials
    QDRANT_URL="https://your-qdrant-cluster-url.qdrant.tech:6333"
    QDRANT_API_KEY="your-qdrant-api-key"

    # Neon Serverless Postgres
    NEON_DATABASE_URL="postgresql://user:password@host:port/dbname"

    # Gemini API Credentials
    GEMINI_API_KEY="your-gemini-api-key"
    GEMINI_BASE_URL="https://generativelanguage.googleapis.com/v1beta"
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

2.  **Install dependencies**:
A `requirements.txt` file will be provided with all necessary packages.
    ```bash
    pip install -r requirements.txt
    ```

## 4. Data Ingestion (One-Time Setup)

Before running the API for the first time, you must populate the Qdrant vector database with the book's content.

1.  **Place the book content** in a file accessible to the project (e.g., create a `data/book.txt` file).

2.  **Run the ingestion script**:
The project will include an ingestion script. You can run it as a Python module.
    ```bash
    python -m src.ingest --source data/book.txt
    ```
    This process may take some time depending on the size of the book and the speed of the embedding model.

## 5. Running the Service

Once the setup and data ingestion are complete, you can run the FastAPI web server.

```bash
# The 'uvicorn' command starts the web server.
# The --reload flag enables hot-reloading for development.
uvicorn src.api.main:app --reload
```

The service will be available at `http://127.0.0.1:8000`.

## 6. Calling the API

You can interact with the API using any HTTP client. Here is an example using `curl`.

### Query the Chatbot

```bash
curl -X POST "http://127.0.0.1:8000/query" \
     -H "Content-Type: application/json" \
     -d 
{
           "query": "What is the main purpose of ROS 2?",
           "user_id": "user123"
         }
```

### Query with Selected Text

```bash
curl -X POST "http://127.0.0.1:8000/query" \
     -H "Content-Type: application/json" \
     -d 
{
           "query": "What does this component do?",
           "user_id": "user123",
           "selected_text": "The differential drive controller is responsible for taking velocity commands and sending them to the wheel joints."
         }
```

### Delete User History

```bash
curl -X DELETE "http://127.0.0.1:8000/history/user123"
```
