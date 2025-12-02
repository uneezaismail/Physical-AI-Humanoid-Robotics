---
id: 1
title: Frontend Design Spec
stage: spec
date: 2025-12-02
surface: CLI
model: claude-sonnet-4-5-20250929
feature: 001-frontend-design
branch: 001-frontend-design
user: uneezaismail
command: /sp.specify
labels: [frontend, design, spec]
links:
  spec: specs/001-frontend-design/spec.md
  ticket:
  adr:
  pr:
files:
- path: specs/001-frontend-design/spec.md
  action: created
- path: specs/001-frontend-design/checklists/requirements.md
  action: created
tests:

---

## Prompt

Frontend Design Specification: Physical AI Textbook

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
   * To visually integrate the account creation process into the high-tech aesthetic, making the setup feel like configuring a robot rather than filling out a form.

## Response snapshot

The Frontend Design Specification for the Physical AI Textbook has been created. All checks passed. The spec is ready for the next phase, which is either `/sp.clarify` or `/sp.plan`.

## Outcome

- ✅ Impact: Frontend design specification successfully created and validated.
- 🧪 Tests: No automated tests were run for this stage.
- 📁 Files: Created spec.md and requirements.md.
- 🔁 Next prompts: /sp.clarify or /sp.plan
- 🧠 Reflection: Successfully generated a comprehensive frontend design specification based on user input, including user scenarios, functional requirements, and success criteria. The spec was validated against quality criteria, and all checks passed. The issue with `create-new-feature.ps1` was mitigated by manually checking out the branch and writing the files.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-new-feature.ps1` script failed due to parameter binding issues.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: N/A
