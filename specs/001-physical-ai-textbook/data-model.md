# Data Model: Physical AI & Humanoid Robotics Textbook

## Entity Definitions

### Chapter
**Description**: A unit of educational content containing concepts, diagrams, code examples, exercises, and reflection questions
**Fields**:
- `id`: Unique identifier for the chapter
- `title`: Chapter title (string)
- `part_id`: Reference to the parent part
- `content`: The MDX content of the chapter (string)
- `learning_outcomes`: List of measurable learning outcomes based on Bloom's Taxonomy (array of strings)
- `prerequisites`: List of prerequisite knowledge required (array of strings)
- `reflection_questions`: List of reflection questions at the end of the chapter (array of strings)
- `teaching_scaffolds`: Teaching aids including "Before you learn this...", "Common misunderstanding...", "Real-world analogy...", "Key takeaway..." (object)
- `word_count`: Estimated word count for the chapter (integer)
- `estimated_reading_time`: Estimated reading time in minutes (integer)
- `created_at`: Timestamp of creation (datetime)
- `updated_at`: Timestamp of last update (datetime)

### Part
**Description**: A collection of related chapters covering a specific area of Physical AI (Foundations, Robotic Nervous System, Digital Twin, AI-Robot Brain, VLA)
**Fields**:
- `id`: Unique identifier for the part
- `title`: Part title (string)
- `description`: Brief description of the part (string)
- `weeks_duration`: Estimated duration in weeks (integer)
- `learning_outcomes`: List of part-level learning outcomes (array of strings)
- `chapters`: List of chapter IDs in this part (array of strings)
- `created_at`: Timestamp of creation (datetime)
- `updated_at`: Timestamp of last update (datetime)

### User
**Description**: A learner with specific hardware capabilities (Tier A: laptop-only, Tier B: Jetson kit, Tier C: humanoid robot)
**Fields**:
- `id`: Unique identifier for the user
- `email`: User's email address (string)
- `name`: User's full name (string)
- `tier`: User tier (enum: A, B, C)
- `hardware_specifications`: Details of user's hardware setup (object)
- `progress`: Learning progress tracking (object with chapter_id and completion status)
- `preferences`: User preferences for learning (object)
- `created_at`: Timestamp of account creation (datetime)
- `updated_at`: Timestamp of last update (datetime)

### Exercise
**Description**: A component that allows user engagement to reinforce learning (thought experiments, coding exercises, reflection questions)
**Fields**:
- `id`: Unique identifier for the exercise
- `type`: Type of exercise (enum: thought_experiment, coding_exercise, reflection_question, hands_on_lab)
- `chapter_id`: Reference to the parent chapter
- `content`: The exercise content (string)
- `solution`: Solution for exercises (string, optional)
- `difficulty`: Difficulty level (enum: beginner, intermediate, advanced)
- `expected_learning`: Learning outcome from completing this exercise (string)
- `created_at`: Timestamp of creation (datetime)

### ContentElement
**Description**: A structured element of textbook content (diagrams, code examples, tables, etc.)
**Fields**:
- `id`: Unique identifier for the content element
- `type`: Type of content element (enum: mermaid_diagram, code_example, table, image, formula)
- `chapter_id`: Reference to the parent chapter
- `content`: The content of the element (string)
- `alt_text`: Alternative text for accessibility (string)
- `caption`: Caption for the element (string, optional)
- `created_at`: Timestamp of creation (datetime)

## Relationships

### Part → Chapter
- One part contains many chapters (1 to many)
- Each chapter belongs to exactly one part
- Foreign key: `part_id` in Chapter entity

### User → Progress
- One user has progress across multiple chapters (1 to many)
- Progress tracking is stored in the User entity's progress field
- Contains chapter_id and completion status

### Chapter → Exercise
- One chapter contains many exercises (1 to many)
- Foreign key: `chapter_id` in Exercise entity

### Chapter → ContentElement
- One chapter contains many content elements (1 to many)
- Foreign key: `chapter_id` in ContentElement entity

## Validation Rules

### Chapter Entity
- `title` must be 5-100 characters
- `content` must follow the required structure (Intro → Learning Objectives → Hook → Concept → Mermaid Diagram → Code Example → Exercises → Summary → Preview Next Chapter)
- `learning_outcomes` must contain 3-5 measurable outcomes
- `word_count` must be between 800-1000
- `estimated_reading_time` must be calculated based on word count (typically 3-4 minutes per 800-1000 words)

### Part Entity
- `title` must be unique
- `weeks_duration` must be between 1-12
- `chapters` must contain at least 1 chapter and follow prerequisite order

### User Entity
- `email` must be unique and valid email format
- `tier` must be one of A, B, or C
- `progress` entries must reference valid chapter IDs

### Exercise Entity
- `type` must be one of the defined exercise types
- `difficulty` must be one of beginner, intermediate, or advanced
- `content` must be provided

### ContentElement Entity
- `type` must be one of the defined content element types
- `alt_text` must be provided for accessibility

## State Transitions

### User Progress
- `not_started` → `in_progress` → `completed`
- Progress is updated when user interacts with chapter content
- Completion is determined by completing required activities in the chapter