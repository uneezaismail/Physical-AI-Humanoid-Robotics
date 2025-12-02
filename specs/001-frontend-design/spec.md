# Feature Specification: Frontend Design Specification: Physical AI Textbook

**Feature Branch**: `001-frontend-design`
**Created**: 2025-12-02
**Status**: Draft
**Input**: User description: "Frontend Design Specification: Physical AI Textbook

  1. Visual Identity & Theme
  What: A "Cyber-Physical" Dark Mode theme utilizing a deep blue-black palette with high-contrast Cyan/Teal accents and Neon Purple highlights.
  Why: To evoke a futuristic, high-tech aesthetic appropriate for robotics and AI, ensuring that code blocks and technical diagrams stand out clearly against
  the background while reducing eye strain during long reading sessions.

  2. Global Navigation
  What: A "Glassmorphism" navigation bar that remains sticky at the top of the page with a semi-transparent, blurred background. It features an enlarged
  project logo and a unique, circular GitHub repository link with a purple glowing ring.
  Why: The glass effect maintains a sense of place within the document by allowing content to scroll "under" the header. The distinct GitHub styling
  prioritizes community engagement and source code access, which is critical for a technical curriculum.

  3. The AI Assistant (Chatbot)

  Entry Point (Floating Action Button)
  What: A prominent, large circular button fixed to the bottom-right corner of the screen. It features a custom "Futuristic Robot" icon and a glowing cyan
  border. On hover, the button animates (bounces/rotates) and the glow intensifies.
  Why: To ensure the AI tutor is immediately discoverable and accessible from any chapter without cluttering the reading interface. The playful animation
  serves as an "invitation" to interact, making the AI feel like an active companion rather than a static tool.

  Chat Interface (The Window)
  What: A floating card overlay that utilizes the same glassmorphism effect as the navbar (semi-transparent background). It sits above the text, allowing the
  reader to vaguely see the book content behind the chat window.
  Why: To prevent the user from feeling "transported" away from the textbook. By keeping the book content visible (though blurred), the UI reinforces that the
  chat is a helper for the current material, not a separate application.

  Conversation Design
  What:
   * User Messages: Aligned to the right, rendered with a transparent background and subtle border (Ghost style).
   * Assistant Messages: Aligned to the left, rendered in a solid, bright Cyan color with dark text.
   * Citations: Source references are appended to the bottom of assistant messages in smaller text.
  Why: To create a clear visual hierarchy. The bright Assistant bubbles instantly draw the eye to the answer, while the transparent User bubbles recede,
  putting focus on the information provided. Citations are visual proof of the "book-grounded" nature of the AI, building trust.

  4. Context-Aware Interaction
  What: A proactive UI element that detects when a user highlights text in the book. A "Context Preview" bar instantly appears attached to the top of the chat
  input area, displaying a snippet of the selected text.
  Why: To mimic the experience of pointing at a paragraph and asking a human tutor, "What does this mean?" It reduces cognitive load by eliminating the need
  for the user to copy-paste context manually.

  5. Input Experience
  What: A pill-shaped text input field accompanied by a circular Send button. The Send button features a purple-to-blue gradient and a glowing shadow.
  Why: The pill shape creates a friendly, modern interface that distinguishes the chat input from standard search bars or forms. The gradient button draws
  attention to the primary action, guiding the user to submit their query.

  6. Feedback & States
  What:
   * Loading: A "bouncing dots" animation displayed within an empty Assistant bubble while waiting for a response.
   * Streaming: Text appears token-by-token rather than all at once.
  Why: To perceive the system as responsive and "thinking." Streaming the text keeps the user engaged immediately, rather than making them wait for the full
  answer to generate, which improves the perceived performance of the RAG system.

  7. Authentication & Personalization
  What:
   * Global Auth Controls: Integrated "Log In" and "Sign Up" buttons in the navigation bar, styled consistently with the glassmorphism theme (Ghost style for Log In, Gradient for Sign Up).
   * Auth Interface: A dedicated login/signup modal or page utilizing the "Cyber-Physical" dark theme (Glassmorphism card, Cyan accents) to match the existing UI.
   * Onboarding Wizard: A post-signup interaction presenting a 5-step questionnaire with graphical selection cards (e.g., Snake icon for Python, Gear for C++) instead of standard forms.
     - Q1 Experience: Toggles visibility of "Prerequisite" and "Math Refresher" boxes.
     - Q2 Language: Toggles standard rclpy vs C++ code tabs.
     - Q3 Hardware: Toggles visibility of "Local Installation" vs "Cloud/AWS Setup" guides.
     - Q4 OS: Prioritizes Docker/WSL2 commands vs Native Linux commands.
     - Q5 Goal: Adjusts the AI Agent's response focus (Math/Physics theory vs Implementation/Hardware setup).
  Why:
   * To provide a tailored educational experience that respects the user's existing knowledge and hardware constraints, preventing frustration (e.g., trying to run Isaac Sim on a MacBook).
   * To gather necessary context for the AI Agent so it can answer questions more relevantly without repeated prompting.
   * To visually integrate the account creation process into the high-tech aesthetic, making the setup feel like configuring a robot rather than filling out a form."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Immersive Dark Mode Experience (Priority: P1)

As a user, I want a visually appealing "Cyber-Physical" Dark Mode theme with high-contrast accents, so that I can read the textbook for extended periods without eye strain and easily distinguish code blocks and diagrams.

**Why this priority**: Essential for the core aesthetic and user comfort during learning.

**Independent Test**: Can be fully tested by navigating to any page and verifying the theme application, contrast, and readability of various content types (text, code, diagrams).

**Acceptance Scenarios**:

1.  **Given** the application is loaded, **When** the dark mode theme is active, **Then** the interface displays a deep blue-black palette with cyan/teal accents and neon purple highlights.
2.  **Given** a page with code blocks and technical diagrams, **When** the dark mode theme is active, **Then** code blocks and diagrams are clearly distinguishable and easy to read.

---

### User Story 2 - Seamless Global Navigation (Priority: P1)

As a user, I want a sticky, semi-transparent navigation bar with a clear project logo and a prominent GitHub link, so that I can easily navigate the textbook and access the project's source code without losing context.

**Why this priority**: Crucial for usability, content discovery, and community engagement.

**Independent Test**: Can be fully tested by scrolling through any chapter and verifying the navigation bar's stickiness, transparency, and the functionality of the logo and GitHub link.

**Acceptance Scenarios**:

1.  **Given** I am scrolling through a chapter, **When** the navigation bar is visible, **Then** it remains fixed at the top with a semi-transparent, blurred background.
2.  **Given** I am on any page, **When** I click the GitHub repository link, **Then** I am directed to the project's GitHub page.

---

### User Story 3 - Accessible AI Assistant Entry Point (Priority: P1)

As a user, I want a clearly visible, interactive floating button for the AI assistant, so that I can easily find and engage with the chatbot from any page to get help with the textbook content.

**Why this priority**: Primary access point for a core feature (the AI chatbot).

**Independent Test**: Can be fully tested by navigating across different pages and verifying the presence, visual styling, and interactive animations of the floating action button.

**Acceptance Scenarios**:

1.  **Given** I am on any page, **When** I view the bottom-right corner of the screen, **Then** I see a large, circular button with a "Futuristic Robot" icon and a glowing cyan border.
2.  **Given** the AI assistant button is visible, **When** I hover over it, **Then** the button animates (bounces/rotates) and its glow intensifies.

---

### User Story 4 - Contextual Chat Interface (Priority: P1)

As a user, I want a chat interface that overlays the textbook content with a glassmorphism effect, so that I can interact with the AI assistant while still seeing the underlying textbook for context.

**Why this priority**: Essential for an integrated and contextual learning experience.

**Independent Test**: Can be fully tested by opening the chat window and verifying its visual appearance (transparency, blur) and how it overlays the textbook content.

**Acceptance Scenarios**:

1.  **Given** the AI assistant chat window is open, **When** I view the window, **Then** it has a semi-transparent, blurred background allowing underlying textbook content to be vaguely visible.
2.  **Given** the chat window is open, **When** I scroll the main textbook content, **Then** the chat window remains in its position relative to the viewport.

---

### User Story 5 - Clear Conversation Flow (Priority: P2)

As a user, I want clear visual distinction between my messages and the AI assistant's responses, with citations for AI answers, so that I can easily follow the conversation and trust the source of information.

**Why this priority**: Improves readability and builds trust in the AI's answers.

**Independent Test**: Can be tested by initiating a conversation with the AI and observing the rendering of user and assistant messages, including citations.

**Acceptance Scenarios**:

1.  **Given** a conversation with the AI assistant, **When** I send a message, **Then** my message is right-aligned with a transparent background and subtle border (Ghost style).
2.  **Given** a conversation with the AI assistant, **When** the assistant responds, **Then** its message is left-aligned in a solid, bright Cyan color with dark text, and includes source citations at the bottom.

---

### User Story 6 - Context-Aware Querying (Priority: P2)

As a user, I want to highlight text in the book and have it appear as a context preview in the chat input, so that I can quickly ask questions about specific parts of the textbook without copying and pasting.

**Why this priority**: Enhances the natural interaction with the AI, making it more efficient.

**Independent Test**: Can be tested by highlighting various sections of text in the textbook and verifying the appearance and content of the context preview bar in the chat input area.

**Acceptance Scenarios**:

1.  **Given** I highlight text within the textbook, **When** the chat interface is open, **Then** a "Context Preview" bar appears above the chat input, displaying the highlighted text snippet.
2.  **Given** a text snippet is displayed in the "Context Preview" bar, **When** I type a question and send it, **Then** the highlighted text is included as context for the AI's response.

---

### User Story 7 - Intuitive Chat Input (Priority: P3)

As a user, I want a visually distinct pill-shaped input field and an attention-grabbing Send button for the chat, so that I can easily enter and submit my questions.

**Why this priority**: Improves usability and visual appeal of the chat interaction.

**Independent Test**: Can be tested by opening the chat interface and observing the styling of the text input and send button.

**Acceptance Scenarios**:

1.  **Given** the chat interface is open, **When** I view the input area, **Then** there is a pill-shaped text input field.
2.  **Given** the chat interface is open, **When** I view the Send button, **Then** it is circular with a purple-to-blue gradient and a glowing shadow.

---

### User Story 8 - Responsive AI Feedback (Priority: P3)

As a user, I want to see loading animations and streaming text from the AI assistant, so that I perceive the system as responsive and engaged during response generation.

**Why this priority**: Enhances perceived performance and user engagement.

**Independent Test**: Can be tested by asking the AI a question and observing the loading and streaming behavior of its response.

**Acceptance Scenarios**:

1.  **Given** I have sent a message to the AI and am awaiting a response, **When** the AI is processing, **Then** a "bouncing dots" animation is displayed within an empty Assistant bubble.
2.  **Given** the AI is generating a response, **When** the response is being sent, **Then** the text appears token-by-token rather than all at once.

---

### User Story 9 - Personalized Learning Experience (Priority: P2)

As a user, I want to create an account and answer questions about my background (experience, hardware, OS, goals), so that the textbook content and AI chatbot responses are tailored to my specific needs and constraints.

**Why this priority**: significantly improves the learning curve and reduces frustration by hiding irrelevant or incompatible instructions.

**Independent Test**: Can be tested by signing up as different personas (e.g., "Beginner with Laptop" vs. "Expert with Workstation") and verifying that the textbook content (e.g., Isaac Sim guides) and AI responses change accordingly.

**Acceptance Scenarios**:

1.  **Given** I am a new user, **When** I click "Sign Up", **Then** I am presented with a "Cyber-Physical" styled modal/page to create an account.
2.  **Given** I have created an account, **When** I am presented with the onboarding questionnaire, **Then** I can select my experience level, programming language, hardware, OS, and goals using graphical cards.
3.  **Given** I have identified as a "Beginner", **When** I view early chapters, **Then** "Prerequisite" and "Math Refresher" boxes are expanded.
4.  **Given** I have identified as using a "Standard Laptop", **When** I view simulation chapters, **Then** "Local Installation" guides are hidden and "Cloud/AWS Setup" is highlighted.

---

### Edge Cases

- What happens when a user highlights an extremely long section of text? (Truncate and indicate truncation)
- How does the system handle rapid-fire queries to the chatbot? (Implement rate limiting and clear visual feedback)
- What if the GitHub repository is private or deleted? (Graceful error handling for the GitHub link)

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: System MUST render a "Cyber-Physical" Dark Mode theme with a deep blue-black palette, high-contrast Cyan/Teal accents, and Neon Purple highlights.
-   **FR-002**: System MUST ensure readability of code blocks and technical diagrams within the Dark Mode theme.
-   **FR-003**: System MUST display a sticky global navigation bar with a glassmorphism effect (semi-transparent, blurred background).
-   **FR-004**: System MUST include an enlarged project logo in the global navigation bar.
-   **FR-005**: System MUST feature a unique, circular GitHub repository link with a purple glowing ring in the global navigation bar.
-   **FR-006**: System MUST present a prominent, large circular floating action button (FAB) in the bottom-right corner of the screen.
-   **FR-007**: System MUST display a custom "Futuristic Robot" icon on the FAB.
-   **FR-008**: System MUST apply a glowing cyan border to the FAB.
-   **FR-009**: System MUST animate the FAB (bounce/rotate) and intensify its glow on hover.
-   **FR-010**: System MUST open a floating card overlay for the chat interface, utilizing a glassmorphism effect (semi-transparent background) that allows vague visibility of underlying textbook content.
-   **FR-011**: System MUST align user messages to the right with a transparent background and subtle border (Ghost style).
-   **FR-012**: System MUST align AI assistant messages to the left, rendered in a solid, bright Cyan color with dark text.
-   **FR-013**: System MUST append source citations in smaller text to the bottom of AI assistant messages.
-   **FR-014**: System MUST detect when a user highlights text in the book.
-   **FR-015**: System MUST display a "Context Preview" bar attached to the top of the chat input area, showing a snippet of the selected text.
-   **FR-016**: System MUST provide a pill-shaped text input field in the chat interface.
-   **FR-017**: System MUST provide a circular Send button with a purple-to-blue gradient and a glowing shadow in the chat interface.
-   **FR-018**: System MUST display a "bouncing dots" animation within an empty Assistant bubble while waiting for an AI response.
-   **FR-019**: System MUST stream AI assistant responses token-by-token.
-   **FR-020**: System MUST display "Log In" (Ghost style) and "Sign Up" (Gradient style) buttons in the global navigation bar.
-   **FR-021**: System MUST provide a login/signup interface that matches the "Cyber-Physical" dark theme (Glassmorphism card, Cyan accents).
-   **FR-022**: System MUST present a 5-step onboarding questionnaire after signup using graphical selection cards.
-   **FR-023**: System MUST capture user data for Experience Level, Programming Language, Hardware, OS, and Main Goal.
-   **FR-024**: System MUST dynamically toggle content visibility (Prerequisites, Math Tips, C++ Code, Installation Guides) based on user profile data.
-   **FR-025**: System MUST persist user preferences and profile data in a database (Neon serverless).
-   **FR-026**: System MUST integrate with `better-auth` for secure authentication.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: Users report a 90% satisfaction rate with the visual theme and readability during extended reading sessions.
-   **SC-002**: Core navigation tasks (chapter switching, GitHub access) are completed 100% of the time without user errors.
-   **SC-003**: The AI assistant FAB is discovered and clicked by 95% of new users within their first 5 minutes of interaction.
-   **SC-004**: Users successfully utilize the context-aware interaction feature in 80% of their chatbot queries when relevant text is highlighted.
-   **SC-005**: Perceived latency for AI responses (from query submission to first token) is under 2 seconds.
-   **SC-006**: 100% of "Standard Laptop" users are correctly routed to "Cloud/AWS Setup" guides, preventing local installation attempts.
