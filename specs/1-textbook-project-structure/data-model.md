# Data Model: Physical AI & Humanoid Robotics Textbook Project

## TextbookModule
**Description**: Represents a major section of the textbook (e.g., ROS 2 Fundamentals, Simulation, etc.)

**Attributes**:
- id: Unique identifier for the module
- title: Display name of the module
- description: Brief overview of the module content
- module_number: Sequential number of the module (1-4)
- estimated_duration: Estimated time to complete in hours
- difficulty_level: Beginner, Intermediate, or Advanced
- prerequisites: Array of prerequisite module IDs
- created_at: Timestamp of creation
- updated_at: Timestamp of last update

**Relationships**:
- Contains many Chapters
- Belongs to a Textbook
- Has many ModuleProgress records

## Chapter
**Description**: Represents a subsection within a module containing specific learning content

**Attributes**:
- id: Unique identifier for the chapter
- title: Display name of the chapter
- content: Markdown content for the chapter
- module_id: Reference to parent module
- chapter_number: Sequential number within the parent module
- estimated_time: Estimated time to complete in minutes
- learning_objectives: Array of learning objectives for the chapter
- requires_simulation: Boolean indicating if simulation examples are included

**Relationships**:
- Belongs to a TextbookModule
- Has many ChapterProgress records
- Has many QuizQuestions

## StudentProfile
**Description**: Represents user information including accessibility preferences and personalization settings

**Attributes**:
- id: Unique identifier for the student
- username: Unique username for the student
- email: Student's email address
- preferred_language: Primary language preference
- accessibility_requirements: JSON object containing accessibility preferences
- urdu_translation_enabled: Boolean for Urdu translation preference
- personalization_quiz_results: JSON object containing quiz responses for personalization
- created_at: Timestamp of account creation
- updated_at: Timestamp of last update

**Relationships**:
- Has many ModuleProgress records
- Has many ChapterProgress records
- Has many RAGQueries

## ModuleProgress
**Description**: Tracks student progress through individual modules

**Attributes**:
- id: Unique identifier for the progress record
- student_id: Reference to the student
- module_id: Reference to the module
- status: Not Started, In Progress, or Completed
- completion_percentage: Percentage of module completed
- time_spent: Total time spent on the module in minutes
- last_accessed: Timestamp of last interaction
- completed_at: Timestamp when module was completed

**Relationships**:
- Belongs to a StudentProfile
- Belongs to a TextbookModule
- Has many ChapterProgress records

## ChapterProgress
**Description**: Tracks student progress through individual chapters

**Attributes**:
- id: Unique identifier for the progress record
- student_id: Reference to the student
- chapter_id: Reference to the chapter
- status: Not Started, In Progress, or Completed
- completion_percentage: Percentage of chapter completed
- time_spent: Total time spent on the chapter in minutes
- last_accessed: Timestamp of last interaction
- quiz_score: Score on chapter quiz if applicable
- completed_at: Timestamp when chapter was completed

**Relationships**:
- Belongs to a StudentProfile
- Belongs to a Chapter

## RAGKnowledgeBase
**Description**: Represents the structured data and vector embeddings used for the Q&A system

**Attributes**:
- id: Unique identifier for the knowledge base entry
- content_id: Reference to the textbook content this entry relates to
- content_type: Module, Chapter, or Specific concept
- content_text: The actual text content for RAG queries
- embeddings: Vector embeddings for semantic search
- source_module: Reference to the module this content belongs to
- source_chapter: Reference to the chapter this content belongs to
- created_at: Timestamp of creation
- updated_at: Timestamp of last update

**Relationships**:
- Associated with content in TextbookModules and Chapters
- Used by many RAGQueries

## RAGQuery
**Description**: Represents a question asked by a student to the RAG system

**Attributes**:
- id: Unique identifier for the query
- student_id: Reference to the student who asked the question
- query_text: The original question text
- response_text: The RAG system's response
- confidence_score: Confidence level of the response (0-1)
- query_time: Timestamp when the query was made
- source_citations: Array of content IDs used to generate the response
- helpfulness_rating: Optional rating provided by the student

**Relationships**:
- Belongs to a StudentProfile
- References multiple RAGKnowledgeBase entries through source_citations

## QuizQuestion
**Description**: Represents a question associated with a specific chapter for assessment

**Attributes**:
- id: Unique identifier for the question
- chapter_id: Reference to the chapter this question belongs to
- question_text: The text of the question
- question_type: Multiple Choice, Short Answer, or True/False
- options: Array of possible answers (for multiple choice)
- correct_answer: The correct answer
- difficulty_level: Easy, Medium, or Hard
- explanation: Explanation of the correct answer
- created_at: Timestamp of creation

**Relationships**:
- Belongs to a Chapter
- Has many QuizSubmissions