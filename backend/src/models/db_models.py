from pydantic import BaseModel, Field
from typing import List
import uuid
from datetime import datetime

class Message(BaseModel):
    id: uuid.UUID = Field(default_factory=uuid.uuid4)
    conversation_id: uuid.UUID
    role: str
    content: str
    created_at: datetime = Field(default_factory=datetime.utcnow)

class Conversation(BaseModel):
    id: uuid.UUID = Field(default_factory=uuid.uuid4)
    user_id: str
    created_at: datetime = Field(default_factory=datetime.utcnow)
    messages: List[Message] = []
