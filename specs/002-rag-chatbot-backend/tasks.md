# Tasks: Integrated RAG Chatbot

**Input**: Design documents from `specs/002-rag-chatbot-backend/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- All paths are relative to the repository root.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure.

- [x] T001 Create the backend directory structure: `backend/src/api`, `backend/src/models`, `backend/src/services`, `backend/tests`.
- [x] T002 [P] Create and populate `backend/pyproject.toml` with initial dependencies: `fastapi`, `uvicorn[standard]`, `pydantic`, `python-dotenv`.
- [x] T003 [P] Create a `backend/.env.example` file with placeholders for all required environment variables: `QDRANT_URL`, `QDRANT_API_KEY`, `NEON_DATABASE_URL`, `GEMINI_API_KEY`, `GEMINI_BASE_URL`.

---

## Phase 2: Foundational Services (Blocking Prerequisites)

**Purpose**: Core services that MUST be complete before ANY user story can be implemented.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

- [x] T004 Implement a service to connect to the Qdrant client in `backend/src/services/rag_service.py`.
- [x] T005 Implement a function `retrieve_context(query: str) -> list[str]` in `backend/src/services/rag_service.py` that queries Qdrant and returns relevant document chunks.
- [x] T006 Implement the core agent invocation logic in `backend/src/services/agent.py`. This service should take a user query and a list of context strings, format them into a prompt, and call the Gemini model via the `openai` SDK.

**Checkpoint**: Core RAG and Agent services are implemented.

---

## Phase 3: User Story 1 - Ask General Questions (Priority: P1) 🎯 MVP

**Goal**: Deliver a functional chatbot that can answer general questions based on the book content by wiring together the foundational services into a usable API endpoint.

**Independent Test**: Call the `/query` endpoint with a JSON payload containing a `query` and verify the response is an accurate, non-empty string derived from the book content.

### Implementation for User Story 1

- [x] T007 [P] [US1] Define Pydantic models for the `/query` request and response bodies in `backend/src/models/api_models.py`.
- [x] T008 [US1] Create the basic FastAPI app instance and set up the `/query` POST endpoint in `backend/src/api/main.py`.
- [x] T009 [US1] In the `/query` endpoint in `backend/src/api/main.py`, integrate the `rag_service.py` and `agent.py` to handle a basic query. The flow is: request query -> `retrieve_context` -> `agent` -> response.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase 4: User Story 2 - Ask Context-Based Questions (Priority: P1)

**Goal**: Enhance the chatbot to understand and use user-selected text for more precise answers.

**Independent Test**: Call the `/query` endpoint with a `query` and a `selected_text` field, and verify the chatbot's response specifically uses the provided context.

### Implementation for User Story 2

- [x] T010 [US2] Modify the agent prompt engineering logic in `backend/src/services/agent.py` to conditionally incorporate the `selected_text` into the prompt that is sent to the Gemini model, giving it higher priority than the retrieved context.
- [x] T011 [US2] Update the `/query` request Pydantic model in `backend/src/models/api_models.py` to include the optional `selected_text` field.

**Checkpoint**: At this point, User Stories 1 and 2 should both work independently.

---

## Phase 5: User Story 3 - Conversation History (Priority: P2)

**Goal**: Enable the chatbot to remember past interactions within a session by connecting to the Neon database.

**Independent Test**: Send a sequence of related questions with the same `user_id` and verify the chatbot uses context from previous turns. Test the `DELETE /history/{user_id}` endpoint and confirm the history is cleared.

### Implementation for User Story 3

- [x] T012 [P] [US3] Implement database connection logic for Neon Postgres using `asyncpg` in `backend/src/services/db.py`.
- [x] T013 [P] [US3] Define data models for `Conversation` and `Message` in `backend/src/models/db_models.py` that align with `data-model.md`.
- [x] T014 [US3] Implement `save_message(user_id, role, content)` and `get_history(user_id)` functions in `backend/src/services/db.py`.
- [x] T015 [US3] In `backend/src/api/main.py`, modify the `/query` endpoint to fetch history before calling the agent and save the new query and response after.
- [x] T016 [US3] In `backend/src/api/main.py`, implement the `DELETE /history/{user_id}` endpoint as defined in the OpenAPI contract.

**Checkpoint**: All user stories should now be independently functional.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

- [x] T017 [P] Implement comprehensive error handling (e.g., for database connection failures, Qdrant timeouts) in all API endpoints in `backend/src/api/main.py`.
- [x] T018 [P] Add structured logging to all services for easier debugging.
- [x] T019 Create a `README.md` file inside the `backend` directory explaining the setup, environment variables, and how to run the service and tests.

---

## Dependencies & Execution Order

- **Phase 1 (Setup)** must complete before all other phases.
- **Phase 2 (Foundational)** depends on Phase 1 and blocks all user stories.
- **Phases 3, 4, 5 (User Stories)** depend on Phase 2. They can be worked on sequentially (US1 -> US2 -> US3) or in parallel if staffed.
- **Phase 6 (Polish)** can be worked on after all desired user stories are complete.

## Implementation Strategy

The recommended approach is **Incremental Delivery**:
1. Complete **Phase 1 and 2** to build the foundation.
2. Complete **Phase 3 (US1)** to achieve the MVP: a functioning RAG chatbot.
3. Complete **Phase 4 (US2)** to add contextual understanding.
4. Complete **Phase 5 (US3)** to add conversation memory.
5. Finish with **Phase 6 (Polish)**.
