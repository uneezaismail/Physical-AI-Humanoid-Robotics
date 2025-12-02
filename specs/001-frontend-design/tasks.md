# Tasks: Frontend Design Specification: Physical AI Textbook

**Input**: Design documents from `/specs/001-frontend-design/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The feature specification does not explicitly request tests to be written as part of the implementation tasks. Therefore, test tasks will not be generated.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Paths shown below assume a web app structure as defined in `plan.md` (frontend/src/)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Configure Docusaurus theme and styling in `frontend/docusaurus.config.ts` and `frontend/src/css/custom.css`
- [x] T002 Update `frontend/package.json` with necessary dependencies (clsx, etc.)
- [x] T003 [P] Configure TypeScript for frontend in `frontend/tsconfig.json`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Define global CSS variables for Cyber-Physical theme in `frontend/src/css/custom.css`
- [x] T005 Implement utility for dynamic class management (e.g., `clsx` integration) in `frontend/src/lib/utils.ts`
- [x] T006 Update `frontend/src/pages/index.tsx` to include basic layout for theme integration.

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Immersive Dark Mode Experience (Priority: P1) 🎯 MVP

**Goal**: As a user, I want a visually appealing "Cyber-Physical" Dark Mode theme with high-contrast accents, so that I can read the textbook for extended periods without eye strain and easily distinguish code blocks and diagrams.

**Independent Test**: Can be fully tested by navigating to any page and verifying the theme application, contrast, and readability of various content types (text, code, diagrams).

### Implementation for User Story 1

- [x] T007 [US1] Apply dark mode styles to base elements (`body`, `html`) in `frontend/src/css/custom.css`
- [x] T008 [P] [US1] Implement high-contrast styles for code blocks in `frontend/src/css/custom.css`
- [x] T009 [P] [US1] Implement high-contrast styles for technical diagrams in `frontend/src/css/custom.css`
- [x] T010 [US1] Integrate theme switching mechanism in `frontend/src/components/ThemeToggle/index.tsx` (new component)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Seamless Global Navigation (Priority: P1)

**Goal**: As a user, I want a sticky, semi-transparent navigation bar with a clear project logo and a prominent GitHub link, so that I can easily navigate the textbook and access the project's source code without losing context.

**Independent Test**: Can be fully tested by scrolling through any chapter and verifying the navigation bar's stickiness, transparency, and the functionality of the logo and GitHub link.

### Implementation for User Story 2

- [x] T011 [US2] Create `Navbar` component in `frontend/src/components/Navbar/index.tsx`
- [x] T012 [P] [US2] Implement glassmorphism effect for `Navbar` in `frontend/src/components/Navbar/styles.module.css`
- [x] T013 [P] [US2] Add enlarged project logo to `Navbar` in `frontend/src/components/Navbar/index.tsx`


**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Accessible AI Assistant Entry Point (Priority: P1)

**Goal**: As a user, I want a clearly visible, interactive floating button for the AI assistant, so that I can easily find and engage with the chatbot from any page to get help with the textbook content.

**Independent Test**: Can be fully tested by navigating across different pages and verifying the presence, visual styling, and interactive animations of the floating action button.

### Implementation for User Story 3

- [x] T016 [US3] Create `FloatingChatButton` component in `frontend/src/components/FloatingChatButton/index.tsx`
- [x] T017 [P] [US3] Implement custom "Futuristic Robot" icon for `FloatingChatButton` in `frontend/src/components/FloatingChatButton/index.tsx`
- [x] T018 [P] [US3] Apply glowing cyan border to `FloatingChatButton` in `frontend/src/components/FloatingChatButton/styles.module.css`
- [x] T019 [P] [US3] Add hover animations (bounce/rotate, glow intensification) to `FloatingChatButton` in `frontend/src/components/FloatingChatButton/styles.module.css`
- [x] T020 [US3] Integrate `FloatingChatButton` into main Docusaurus layout in `frontend/src/theme/Layout/index.tsx` (override theme component)

**Checkpoint**: At this point, User Stories 1, 2, AND 3 should all work independently

---

## Phase 6: User Story 4 - Contextual Chat Interface (Priority: P1)

**Goal**: As a user, I want a chat interface that overlays the textbook content with a glassmorphism effect, so that I can interact with the AI assistant while still seeing the underlying textbook for context.

**Independent Test**: Can be fully tested by opening the chat window and verifying its visual appearance (transparency, blur) and how it overlays the textbook content.

### Implementation for User Story 4

- [x] T021 [US4] Create `ChatWidget` component in `frontend/src/components/ChatWidget/index.tsx`
- [x] T022 [P] [US4] Implement glassmorphism effect for `ChatWidget` background in `frontend/src/components/ChatWidget/styles.module.css`
- [x] T023 [US4] Ensure `ChatWidget` floats as an overlay above textbook content in `frontend/src/components/ChatWidget/styles.module.css`

**Checkpoint**: At this point, User Stories 1, 2, 3, AND 4 should all work independently

---

## Phase 7: User Story 5 - Clear Conversation Flow (Priority: P2)

**Goal**: As a user, I want clear visual distinction between my messages and the AI assistant's responses, with citations for AI answers, so that I can easily follow the conversation and trust the source of information.

**Independent Test**: Can be tested by initiating a conversation with the AI and observing the rendering of user and assistant messages, including citations.

### Implementation for User Story 5

- [x] T024 [US5] Style user messages (right-aligned, transparent background, subtle border) in `frontend/src/components/ChatWidget/styles.module.css`
- [x] T025 [US5] Style AI assistant messages (left-aligned, solid Cyan background, dark text) in `frontend/src/components/ChatWidget/styles.module.css`
- [x] T026 [US5] Implement rendering of source citations in smaller text at the bottom of AI messages in `frontend/src/components/ChatWidget/index.tsx`

---

## Phase 8: User Story 6 - Context-Aware Querying (Priority: P2)

**Goal**: As a user, I want to highlight text in the book and have it appear as a context preview in the chat input, so that I can quickly ask questions about specific parts of the textbook without copying and pasting.

**Independent Test**: Can be tested by highlighting various sections of text in the textbook and verifying the appearance and content of the context preview bar in the chat input area.

### Implementation for User Story 6

- [x] T027 [US6] Implement text highlighting detection logic in `frontend/src/hooks/useHighlightDetection.ts` (new hook)
- [x] T028 [US6] Create `ContextPreviewBar` component in `frontend/src/components/ChatWidget/ContextPreviewBar.tsx` (new component)
- [x] T029 [US6] Integrate `ContextPreviewBar` into `ChatWidget` and display highlighted text snippet in `frontend/src/components/ChatWidget/index.tsx`
- [x] T030 [US6] Modify chat input submission to include highlighted text as context in `frontend/src/lib/api.ts` (or `frontend/src/services/chat.ts`)

---

## Phase 9: User Story 9 - Personalized Learning Experience (Priority: P2)

**Goal**: As a user, I want to create an account and answer questions about my background (experience, hardware, OS, goals), so that the textbook content and AI chatbot responses are tailored to my specific needs and constraints.

**Independent Test**: Can be tested by signing up as different personas (e.g., "Beginner with Laptop" vs. "Expert with Workstation") and verifying that the textbook content (e.g., Isaac Sim guides) and AI responses change accordingly.

### Implementation for User Story 9

- [x] T031 [US9] Implement "Log In" (Ghost style) and "Sign Up" (Gradient style) buttons in `frontend/src/components/Navbar/index.tsx`
- [x] T032 [US9] Create `AuthModal` component for login/signup interface with Cyber-Physical theme in `frontend/src/components/AuthModal/index.tsx`
- [x] T033 [US9] Create `OnboardingWizard` component for 5-step questionnaire with graphical cards in `frontend/src/components/OnboardingWizard/index.tsx`
- [x] T034 [US9] Develop frontend logic to capture user profile data (experience, language, hardware, OS, goal) from `OnboardingWizard` in `frontend/src/lib/userProfileService.ts` (new service)
- [x] T035 [US9] Implement dynamic content toggling logic based on user profile in `frontend/src/lib/contentVisibility.ts` (new utility) and integrate into Docusaurus pages.
- [x] T036 [US9] Integrate frontend with `better-auth` backend for user authentication in `frontend/src/lib/api.ts` (or `frontend/src/services/auth.ts`)
- [x] T037 [US9] Implement API calls to persist user preferences and profile data to Neon serverless database (via backend) in `frontend/src/lib/api.ts` (or `frontend/src/services/userProfile.ts`)

---

## Phase 10: User Story 7 - Intuitive Chat Input (Priority: P3)

**Goal**: As a user, I want a visually distinct pill-shaped input field and an attention-grabbing Send button for the chat, so that I can easily enter and submit my questions.

**Independent Test**: Can be tested by opening the chat interface and observing the styling of the text input and send button.

### Implementation for User Story 7

- [x] T038 [US7] Style pill-shaped text input field in `frontend/src/components/ChatWidget/styles.module.css`
- [x] T039 [US7] Style circular Send button with purple-to-blue gradient and glowing shadow in `frontend/src/components/ChatWidget/styles.module.css`

---

## Phase 11: User Story 8 - Responsive AI Feedback (Priority: P3)

**Goal**: As a user, I want to see loading animations and streaming text from the AI assistant, so that I perceive the system as responsive and engaged during response generation.

**Independent Test**: Can be tested by asking the AI a question and observing the loading and streaming behavior of its response.

### Implementation for User Story 8

- [x] T040 [US8] Implement "bouncing dots" loading animation within empty Assistant bubble in `frontend/src/components/ChatWidget/index.tsx` and `frontend/src/components/ChatWidget/styles.module.css`
- [x] T041 [US8] Integrate streaming text display for AI responses (token-by-token) in `frontend/src/components/ChatWidget/index.tsx`

---

## Phase 12: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T042 Implement truncation logic for long highlighted text snippets in `frontend/src/hooks/useHighlightDetection.ts`
- [x] T043 Implement frontend rate limiting and clear visual feedback for rapid-fire chatbot queries in `frontend/src/lib/api.ts` and `frontend/src/components/ChatWidget/index.tsx`
- [x] T043 Implement frontend rate limiting and clear visual feedback for rapid-fire chatbot queries in `frontend/src/lib/api.ts` and `frontend/src/components/ChatWidget/index.tsx`
- [x] T043 Implement frontend rate limiting and clear visual feedback for rapid-fire chatbot queries in `frontend/src/lib/api.ts` and `frontend/src/components/ChatWidget/index.tsx`
- [x] T044 Implement graceful error handling for the GitHub repository link (e.g., if private or deleted) in `frontend/src/components/Navbar/index.tsx`
- [x] T045 Ensure overall mobile responsiveness (320px width minimum) across all implemented components in `frontend/src/css/custom.css` and individual component CSS modules.
- [x] T046 Conduct accessibility audit and fix any issues (WCAG AA compliance) across all new UI elements.
- [x] T047 Conduct performance audit and optimize (e.g., lazy loading, image optimization) for all new UI elements.

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately
-   **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
-   **User Stories (Phase 3+)**: All depend on Foundational phase completion
    -   User stories can then proceed in parallel (if staffed)
    -   Or sequentially in priority order (P1 → P2 → P3)
-   **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

-   **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
-   **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
-   **User Story 3 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
-   **User Story 4 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
-   **User Story 5 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
-   **User Story 6 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
-   **User Story 9 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
-   **User Story 7 (P3)**: Can start after Foundational (Phase 2) - No dependencies on other stories
-   **User Story 8 (P3)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

-   Models before services
-   Services before endpoints
-   Core implementation before integration
-   Story complete before moving to next priority

### Parallel Opportunities

-   All Setup tasks marked [P] can run in parallel
-   All Foundational tasks marked [P] can run in parallel (within Phase 2)
-   Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
-   All tests for a user story marked [P] can run in parallel
-   Models within a story marked [P] can run in parallel
-   Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Example of parallelizable tasks within User Story 1:
# These tasks are independent of each other and can be worked on concurrently.

# Implement high-contrast styles for code blocks
Task: "Implement high-contrast styles for code blocks in frontend/src/css/custom.css"
# Implement high-contrast styles for technical diagrams
Task: "Implement high-contrast styles for technical diagrams in frontend/src/css/custom.css"
```

---

## Implementation Strategy

### MVP First (User Stories with P1 only)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3.  Complete Phase 3: User Story 1 (Immersive Dark Mode)
4.  Complete Phase 4: User Story 2 (Seamless Global Navigation)
5.  Complete Phase 5: User Story 3 (Accessible AI Assistant Entry Point)
6.  Complete Phase 6: User Story 4 (Contextual Chat Interface)
7.  **STOP and VALIDATE**: Test User Stories 1-4 independently
8.  Deploy/demo if ready

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready
2.  Add User Story 1 (P1) → Test independently → Deploy/Demo
3.  Add User Story 2 (P1) → Test independently → Deploy/Demo
4.  Add User Story 3 (P1) → Test independently → Deploy/Demo
5.  Add User Story 4 (P1) → Test independently → Deploy/Demo
6.  Add User Story 5 (P2) → Test independently → Deploy/Demo
7.  Add User Story 6 (P2) → Test independently → Deploy/Demo
8.  Add User Story 9 (P2) → Test independently → Deploy/Demo
9.  Add User Story 7 (P3) → Test independently → Deploy/Demo
10. Add User Story 8 (P3) → Test independently → Deploy/Demo
11. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together
2.  Once Foundational is done:
    -   Developer A: User Story 1, 2, 3, 4 (P1s)
    -   Developer B: User Story 5, 6, 9 (P2s)
    -   Developer C: User Story 7, 8 (P3s)
3.  Stories complete and integrate independently

---

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
