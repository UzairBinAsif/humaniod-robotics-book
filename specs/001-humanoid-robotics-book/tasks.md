# Tasks: Physical AI & Humanoid Robotics Book

**Input**: Design documents from `specs/001-humanoid-robotics-book/`
**Prerequisites**: plan.md, spec.md

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel
- **[Story]**: Which user story this task belongs to (US1: Read Content, US2: Access Code)
- Include exact file paths in descriptions.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize the book's website structure and the code workspaces.

- [x] T001 Initialize Docusaurus project in `book/`.
- [x] T002 [P] Create placeholder directory `part1-ros2` in `book/docs/`.
- [x] T003 [P] Create placeholder directory `part2-simulation` in `book/docs/`.
- [x] T004 [P] Create placeholder directory `part3-isaac` in `book/docs/`.
- [x] T005 [P] Create placeholder directory `part4-vlm` in `book/docs/`.
- [x] T006 [P] Initialize empty ROS 2 workspace in `code/lab1_ws/`.
- [x] T007 [P] Initialize empty ROS 2 workspace in `code/lab2_ws/`.
- [x] T008 [P] Initialize empty ROS 2 workspace in `code/lab3_ws/`.
- [x] T009 [P] Initialize empty ROS 2 workspace in `code/lab4_ws/`.

---

## Phase 2: User Stories 1 & 2 (Part 1 & 2) 🎯 MVP

**Goal**: Write the first half of the book (ROS 2 & Simulation) and implement the corresponding labs.
**Independent Test**: The first two parts of the book are readable online, and the code for the first two labs can be successfully built and run.

### Implementation for Part 1 & 2

- [x] T010 [US1] Write Chapter "Introduction to Physical AI" in `book/docs/part1-ros2/01-introduction.md`.
- [x] T011 [US1] Write Chapter "ROS 2 Communication Patterns" in `book/docs/part1-ros2/02-communication-patterns.md`.
- [x] T012 [US1] Write Chapter "Humanoid URDF Modeling" in `book/docs/part1-ros2/03-urdf-modeling.md`.
- [x] T013 [US2] Implement Lab 1 code (Publisher/Subscriber and URDF visualization) in `code/lab1_ws/`.
- [x] T014 [US1] Write Chapter "Simulation Physics in Gazebo & Isaac Sim" in `book/docs/part2-simulation/01-sim-physics.md`.
- [x] T015 [US1] Write Chapter "Isaac Sim Setup" in `book/docs/part2-simulation/02-isaac-setup.md`.
- [x] T016 [US2] Implement Lab 2 code (Simulating the robot in Isaac Sim) in `code/lab2_ws/`.

**Checkpoint**: The core concepts of ROS 2 and Simulation are documented and have functional labs. This represents a valuable, testable MVP.

---

## Phase 3: User Stories 1 & 2 (Part 3 & 4)

**Goal**: Write the second half of the book (AI/ML & VLM) and implement the corresponding labs.
**Independent Test**: The last two parts of the book are readable online, and the code for the last two labs can be successfully built and run.

### Implementation for Part 3 & 4

- [x] T017 [US1] Write Chapter "Navigation with Nav2" in `book/docs/part3-isaac/01-navigation.md`.
- [x] T018 [US2] Implement Lab 3 code (Autonomous navigation with Isaac ROS) in `code/lab3_ws/`.
- [x] T019 [US1] Write Chapter "LLM-to-Action Mapping" in `book/docs/part4-vlm/01-mapping.md`.
- [x] T020 [US2] Implement Lab 4 code (Voice-to-Action pipeline) in `code/lab4_ws/`.

**Checkpoint**: All four parts of the book are written, and all labs are implemented.

---

## Phase 4: Polish & Cross-Cutting Concerns

**Purpose**: Finalize the book and prepare for release.

- [x] T021 [US1] Write "Final Capstone Guide" chapter in `book/docs/05-capstone.md`.
- [x] T022 [P] Review and edit all content in `book/docs/` for clarity, consistency, and technical accuracy.
- [x] T023 [P] Review and refactor all lab code in `code/` for clarity and adherence to best practices.
- [x] T024 Write top-level README.md for the project.
- [x] T025 Build and deploy the final Docusaurus website.

---

## Dependencies & Execution Order

- **Setup (Phase 1)** must be completed before any other phase.
- **Phase 2 and 3** can technically proceed in parallel if multiple authors are available, but a sequential approach is recommended to ensure a consistent narrative.
- Within each content creation phase, writing the chapter ([US1]) should precede implementing the lab ([US2]).
- **Polish (Phase 4)** depends on all previous phases being complete.

---

## Implementation Strategy

The project will follow an incremental delivery model based on the book's parts.

1.  **MVP (Phase 1 & 2)**: Complete the setup, write the first two parts of the book, and implement the first two labs. This provides a solid foundation and delivers value early.
2.  **V2 (Phase 3)**: Complete the final two parts of the book and their corresponding labs.
3.  **Final Release (Phase 4)**: Perform a final review, write the capstone, and publish the website.
