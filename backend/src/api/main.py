import logging
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from src.models.api_models import QueryRequest, QueryResponse
from src.services import rag_service, agent_service, db_service

logger = logging.getLogger(__name__)

app = FastAPI(
    title="Integrated RAG Chatbot API",
    description="API for querying the book-focused RAG chatbot and managing user history.",
    version="1.0.0",
)

# CORS Middleware
origins = [
    "http://localhost:3000",  # Docusaurus default port
    "https://humaniod-robotics-book-xi.vercel.app",  # Production frontend URL
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_origin_regex=r"https://.*\.vercel\.app",  # Allow all Vercel preview domains
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.exception_handler(Exception)
async def generic_exception_handler(request: Request, exc: Exception):
    # In a real application, you would log this exception.
    print(f"Unhandled exception: {exc}")
    return JSONResponse(
        status_code=500,
        content={"message": "An unexpected error occurred."}
    )


@app.on_event("startup")
async def startup():
    await db_service.connect()

@app.on_event("shutdown")
async def shutdown():
    await db_service.disconnect()


@app.post("/query", response_model=QueryResponse)
async def query_chatbot(request: QueryRequest):
    """
    Sends a query, an optional user ID, and optional selected text to the RAG chatbot and receives a response.
    """
    if not request.query:
        raise HTTPException(status_code=400, detail="Query cannot be empty.")
    
    try:
        history = []
        if request.user_id:
            history = await db_service.get_history(request.user_id)

        # 1. Retrieve context from RAG service
        context = rag_service.retrieve_context(request.query)
        
        # 2. Invoke agent with query, context, and history
        response_text = agent_service.invoke(request.query, context, request.selected_text, history)
        
        if request.user_id:
            await db_service.save_message(request.user_id, "user", request.query)
            await db_service.save_message(request.user_id, "assistant", response_text)

        return QueryResponse(response=response_text)
    except Exception as e:
        error_msg = str(e)
        logger.error(f"An unexpected error occurred during query processing: {e}")
        # Check for rate limit errors
        if "429" in error_msg or "RESOURCE_EXHAUSTED" in error_msg or "quota" in error_msg.lower():
            raise HTTPException(status_code=429, detail="API rate limit exceeded. Please try again later.")
        raise HTTPException(status_code=503, detail="The service is unavailable or encountered an error.")

@app.get("/")
async def root():
    return {"message": "Welcome to the RAG Chatbot API"}

@app.delete("/history/{user_id}")
async def delete_user_history(user_id: str):
    """
    Deletes all conversation history associated with a specific user ID.
    """
    try:
        deleted = await db_service.delete_history(user_id)
        if not deleted:
            raise HTTPException(status_code=404, detail="User not found.")
        
        return {"message": f"History for user {user_id} has been deleted."}
    except Exception as e:
        print(f"Error during history deletion: {e}")
        raise HTTPException(status_code=503, detail="Service is unavailable or encountered an error.")
