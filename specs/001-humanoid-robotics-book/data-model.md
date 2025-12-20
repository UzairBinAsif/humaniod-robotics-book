# Data Model: Humanoid Robotics Book

This document defines the key data entities for structuring the book content and code.

## Entity Definitions

### 1. Book

The top-level container for the entire project.

-   **Fields**:
    -   `title` (string): "Physical AI & Humanoid Robotics"
    -   `authors` (list[string]): A list of contributing authors.
    -   `version` (string): The book's version number.
-   **Relationships**:
    -   Has four `Parts`.

### 2. Part

A major section of the book, corresponding to one of the four core modules.

-   **Fields**:
    -   `title` (string): e.g., "The Nervous System (ROS 2)"
    -   `part_number` (integer): 1 through 4.
-   **Relationships**:
    -   Belongs to one `Book`.
    -   Contains multiple `Chapters`.
    -   Contains one `Lab`.

### 3. Chapter

A thematic chapter within a Part.

-   **Fields**:
    -   `title` (string): e.g., "Describing Robots with URDF"
    -   `chapter_number` (integer): The sequential number within the Part.
    -   `content` (Markdown): The textual content of the chapter.
-   **Relationships**:
    -   Belongs to one `Part`.
    -   May contain multiple `SubChapters`.

### 4. SubChapter

An optional, more granular topic within a Chapter.

-   **Fields**:
    -   `title` (string): e.g., "Understanding Links, Joints, and Visuals"
    -   `content` (Markdown): The textual content.
-   **Relationships**:
    -   Belongs to one `Chapter`.

### 5. Lab

A hands-on practical exercise concluding a Part.

-   **Fields**:
    -   `title` (string): e.g., "Hands-on Lab 1"
    -   `objective` (string): A brief description of the lab's goal.
    -   `task_description` (Markdown): Detailed steps to complete the lab.
-   **Relationships**:
    -   Belongs to one `Part`.
    -   Associated with a specific `ROS 2 Workspace` in the `code/` directory.
