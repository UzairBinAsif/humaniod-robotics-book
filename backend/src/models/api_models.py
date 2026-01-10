from pydantic import BaseModel
from typing import Optional

class QueryRequest(BaseModel):
    """
    Pydantic model for the /query request body.
    """
    query: str
    user_id: Optional[str] = None
    selected_text: Optional[str] = None

class QueryResponse(BaseModel):
    """
    Pydantic model for the /query response body.
    """
    response: str
