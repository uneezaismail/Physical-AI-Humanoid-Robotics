# Feature Specification: Interactive Urdu Translation Button

**Feature Branch**: `002-urdu-translation`
**Created**: 2025-12-18
**Status**: Draft
**Input**: User description: "write specification for urdu translation in which user can translate the content in Urdu in the chapters by pressing a button at the start of each chapter."

## Clarifications

### Session 2025-12-18

- Q: Which URL sharing behavior should be implemented? → A: Shared URLs include language parameter (`?lang=ur`) - recipients see sender's language choice
- Q: Where exactly should the translation button be positioned? → A: Below chapter title, before table of contents / content - balances visibility with hierarchy
- Q: How should the system handle chapters without Urdu translations? → A: Show disabled button with tooltip/message "Urdu translation coming soon" - clear communication, sets expectations

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
  
  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Quick Chapter Translation (Priority: P1)

An Urdu-speaking student visits a chapter page and wants to read the content in their native language. They see a translation button positioned directly below the chapter title, click it once, and the entire chapter content instantly switches to Urdu while maintaining all formatting, code examples, and navigation structure.

**Why this priority**: This is the core functionality that enables Urdu speakers to access educational content in their preferred language, directly addressing the primary user need and delivering immediate value.

**Independent Test**: Can be fully tested by navigating to any chapter, clicking the translation button, and verifying that content displays in Urdu with proper RTL layout and all original formatting preserved. Delivers complete translation experience for a single chapter.

**Acceptance Scenarios**:

1. **Given** a user is viewing a chapter in English, **When** they click the "Translate to Urdu" button at the chapter start, **Then** the chapter content displays in Urdu with RTL (right-to-left) text direction
2. **Given** a user has translated a chapter to Urdu, **When** they click the button again (now showing "Translate to English"), **Then** the content switches back to English with LTR (left-to-right) text direction
3. **Given** a user translates a chapter to Urdu, **When** they navigate to another chapter, **Then** the new chapter displays in the user's previously selected language (Urdu)
4. **Given** a chapter contains code blocks, **When** a user switches to Urdu, **Then** the explanatory text is in Urdu but code blocks remain in original English
5. **Given** a chapter contains images with captions, **When** a user switches to Urdu, **Then** the captions display in Urdu while image paths remain unchanged

---

### User Story 2 - Persistent Language Preference (Priority: P2)

A user who regularly reads content in Urdu wants their language choice to persist across browsing sessions. When they return to the site days later, their preferred language (Urdu) is automatically applied without needing to re-select it on every visit.

**Why this priority**: Enhances user experience by remembering preferences, reducing friction for returning users, and demonstrating respect for user choices. While valuable, it's secondary to the core translation functionality.

**Independent Test**: Can be tested independently by selecting Urdu, closing the browser, returning to the site, and verifying the language preference persists. Delivers sustained convenience without requiring P1 to be complete.

**Acceptance Scenarios**:

1. **Given** a user selects Urdu as their language, **When** they close and reopen their browser, **Then** the site automatically displays in Urdu
2. **Given** a user has set Urdu as their preference, **When** they visit the site from a different device without logging in, **Then** the site displays in default English (preference is device-specific)
3. **Given** a user clears their browser data, **When** they revisit the site, **Then** the language preference resets to default English

---

### User Story 3 - Visual Translation State Indication (Priority: P3)

A user navigating between chapters wants clear visual feedback about which language is currently active. The translation button displays the current language state ("English" or "اردو") and the action it will perform ("Translate to Urdu" or "Translate to English").

**Why this priority**: Improves usability through clear affordances and reduces confusion, but the basic functionality works without sophisticated visual states. This is a polish feature that enhances but doesn't fundamentally change the experience.

**Independent Test**: Can be tested by observing button state changes during language switches and verifying tooltip/label text updates. Delivers clarity without requiring other stories.

**Acceptance Scenarios**:

1. **Given** a chapter is displayed in English, **When** a user hovers over the translation button, **Then** the button shows "Translate to Urdu" or displays Urdu language icon
2. **Given** a chapter is displayed in Urdu, **When** a user hovers over the translation button, **Then** the button shows "Translate to English" or displays English language icon
3. **Given** a user clicks the translation button, **When** the content is switching languages, **Then** a loading indicator briefly appears to acknowledge the action

---

### Edge Cases

- What happens when a chapter has not yet been translated to Urdu?
  - The button appears in a disabled (grayed out, non-clickable) state
  - A tooltip or inline message displays "Urdu translation coming soon" to set user expectations
  - This maintains UI consistency (button always present) while clearly communicating translation status

- How does the system handle partially translated chapters?
  - The button remains clickable
  - Translated sections display in Urdu
  - Untranslated sections remain in English with a subtle indicator

- What happens when a user has JavaScript disabled?
  - The site should fall back to serving the default language (English)
  - Optionally, provide a server-side language selection method via query parameter or subdomain

- How does the translation button behave on mobile devices?
  - The button remains accessible and sized appropriately for touch targets (minimum 44x44px)
  - On smaller screens, the button may show only an icon with language code (e.g., "UR" / "EN")

- What happens when a user shares a URL while viewing Urdu content?
  - The shared URL MUST include language preference (e.g., `?lang=ur`) so recipients see the same language as the sender
  - This ensures consistency when sharing educational content, especially when instructors share specific language versions with students

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display a translation toggle button immediately below the chapter title and before the table of contents or main content (whichever comes first)
- **FR-002**: System MUST switch all chapter content (headings, paragraphs, lists, captions) to Urdu when the translation button is activated
- **FR-003**: System MUST preserve right-to-left (RTL) text direction for Urdu content and left-to-right (LTR) for English
- **FR-004**: System MUST maintain original formatting (markdown structure, code blocks, links, images) during language switching
- **FR-005**: System MUST NOT translate code blocks, code snippets, or technical command syntax
- **FR-006**: System MUST NOT translate file paths, URLs, or import statements
- **FR-007**: System MUST remember the user's language preference across page navigation within the same session
- **FR-008**: System MUST store the user's language preference locally to persist across browser sessions
- **FR-009**: System MUST update the button label or icon to reflect the current language and available translation action
- **FR-010**: System MUST serve pre-translated Urdu content files rather than performing real-time machine translation
- **FR-011**: System MUST handle chapters without available Urdu translations by displaying a disabled (non-clickable) button with a tooltip or message stating "Urdu translation coming soon"
- **FR-012**: System MUST update the page's HTML `lang` attribute (e.g., `lang="ur"` or `lang="en"`) when language changes
- **FR-013**: System MUST update the site's text direction CSS (e.g., `dir="rtl"` or `dir="ltr"`) when language changes
- **FR-014**: Users MUST be able to switch between languages an unlimited number of times without page reload
- **FR-015**: System MUST include language preference in shareable URLs via query parameter (e.g., `?lang=ur`), ensuring recipients see the same language as the sender regardless of their stored preference

### Key Entities *(include if feature involves data)*

- **Chapter**: A single educational unit with title, content sections, code examples, and metadata; exists in both English and Urdu versions
- **Translation Button**: Interactive UI element with state (active language, available translation), position (chapter start), and behavior (toggle language)
- **Language Preference**: User setting stored locally, indicating preferred display language (English or Urdu), persisted across sessions
- **Content File**: Pre-translated markdown file containing chapter content in either English or Urdu, structured identically for seamless switching

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can switch from English to Urdu (or vice versa) within 2 seconds of clicking the translation button
- **SC-002**: Language preference persists across browser sessions with 100% reliability for users who don't clear browser data
- **SC-003**: All translated chapters display with correct RTL layout (sidebar right, content flows RTL) for Urdu without layout breaks
- **SC-004**: Code blocks and technical syntax remain in English with 100% accuracy across all translated chapters
- **SC-005**: 95% of users successfully identify and use the translation button on their first chapter visit (measured via usability testing)
- **SC-006**: Translation switching works consistently across all major browsers (Chrome, Firefox, Safari, Edge) and mobile devices
- **SC-007**: Shared URLs with language parameters (`?lang=ur`) correctly display content in the specified language for 100% of recipients
- **SC-008**: Pages load in the user's preferred language without visible flashing or content shifting (< 100ms language detection)
