# Data Model for Integrated RAG Chatbot

This document defines the data models for the features described in `spec.md`. The primary data requirement is to store conversation history. The models are designed for a relational database (PostgreSQL) and will be managed by the backend service.

## 1. `Conversation`

Represents a single, continuous chat session with a user.

| Field | Type | Description | Constraints |
|---|---|---|---|
| `id` | `UUID` | A unique identifier for the conversation. | Primary Key, Not Null |
| `user_id` | `String` | A unique identifier for the user or session. Used to group conversations for a single user. | Not Null, Indexed |
| `created_at` | `Timestamp` | The timestamp when the conversation was initiated. | Not Null, Default: `NOW()` |

### Example

```json
{
  "id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
  "user_id": "session_xyz789",
  "created_at": "2025-12-29T10:00:00Z"
}
```

## 2. `Message`

Represents a single message within a `Conversation`. Each conversation will consist of multiple messages from both the user and the assistant.

| Field | Type | Description | Constraints |
|---|---|---|---|
| `id` | `UUID` | A unique identifier for the message. | Primary Key, Not Null |
| `conversation_id` | `UUID` | A foreign key linking this message to its `Conversation`. | Not Null, Foreign Key to `Conversation.id`, Indexed |
| `role` | `String` | The originator of the message. Must be either "user" or "assistant". | Not Null, Enum("user", "assistant") |
| `content` | `Text` | The full text content of the message. | Not Null |
| `created_at` | `Timestamp` | The timestamp when the message was created. | Not Null, Default: `NOW()` |

### Example

```json
{
  "id": "f0e9d8c7-b6a5-4321-fedc-ba0987654321",
  "conversation_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
  "role": "user",
  "content": "What is a URDF file?",
  "created_at": "2025-12-29T10:00:15Z"
}
```

## Relationships

- A `Conversation` has one or more `Message` records.
- A `Message` belongs to exactly one `Conversation`.

This simple two-table schema is sufficient to meet the feature requirements of storing and retrieving chat history for users.
