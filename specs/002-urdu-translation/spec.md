# Feature Specification: Docusaurus Native Urdu Translation

**Feature Branch**: `002-urdu-translation`
**Created**: 2025-12-18
**Status**: Implemented
**Approach**: Native Docusaurus i18n with locale paths

## Overview

Implement multilingual support for the Physical AI textbook documentation using Docusaurus's built-in internationalization (i18n) system. The textbook content (docs) will be available in both English and Urdu, while the website UI (homepage, auth, navigation) remains in English only.

## Clarifications

### Design Decisions

- **Q**: Should the entire website be translated or just the docs?
  - **A**: Docs-only translation - homepage, auth pages, and navigation remain English-only. Only the educational content (book chapters) is available in Urdu.

- **Q**: Which i18n approach to use - custom components or native Docusaurus?
  - **A**: Native Docusaurus i18n with locale paths (`/ur/docs/...`) using built-in `localeDropdown` component. This is the standard, maintainable approach.

- **Q**: Should language preferences auto-redirect users?
  - **A**: No auto-redirect. Users stay on their current page type when switching languages (homepage stays homepage, docs stay docs). This provides predictable, user-controlled behavior.

## User Scenarios & Testing

### User Story 1 - Access Urdu Documentation (Priority: P1)

An Urdu-speaking student wants to read the textbook in their native language. They visit the docs, see the language selector in the navbar, click "اردو (Urdu)", and are taken to the Urdu version of the same chapter with RTL layout and translated content.

**Acceptance Scenarios**:

1. **Given** a user is viewing an English chapter at `/docs/part-1/chapter-01`, **When** they click the language dropdown and select "اردو", **Then** they are navigated to `/ur/docs/part-1/chapter-01` with Urdu content and RTL layout
2. **Given** a user is viewing Urdu docs at `/ur/docs/...`, **When** they navigate between chapters, **Then** they remain in the Urdu locale throughout their browsing
3. **Given** a user clicks the language dropdown on a docs page, **When** they select English, **Then** they are navigated to the English version of the same chapter at `/docs/...`
4. **Given** a chapter contains code blocks, **When** displayed in Urdu, **Then** the explanatory text is in Urdu but code remains in English
5. **Given** a user views Urdu docs, **When** they check the sidebar, **Then** chapter titles and category names display in Urdu

---

### User Story 2 - Navigate Multilingual Site (Priority: P1)

A user wants to seamlessly navigate between English and Urdu documentation while maintaining context. The site uses standard URL-based locales (`/ur/`) that work consistently across all browsers and are shareable.

**Acceptance Scenarios**:

1. **Given** a user is on the homepage at `/`, **When** they click the language dropdown, **Then** they see "English" is selected and "اردو" is available
2. **Given** a user is on the homepage, **When** they select "اردو" from the dropdown, **Then** they stay on the homepage (no redirect), as homepage is English-only
3. **Given** a user navigates to Textbook from homepage, **When** they had previously selected Urdu, **Then** they see `/ur/docs/...` with Urdu content
4. **Given** a user receives a shared link to `/ur/docs/part-1/chapter-01`, **When** they open it, **Then** they see the Urdu version regardless of their preference
5. **Given** a user is on auth page `/auth`, **When** they use the language dropdown, **Then** they remain on `/auth` (auth pages are English-only)

---

### User Story 3 - RTL Layout Support (Priority: P2)

Urdu readers expect proper right-to-left text direction and mirrored layouts for comfortable reading. The system automatically applies RTL CSS when displaying Urdu content.

**Acceptance Scenarios**:

1. **Given** a user views `/ur/docs/...`, **When** the page loads, **Then** the HTML has `dir="rtl"` and `lang="ur-PK"` attributes
2. **Given** Urdu docs are displayed, **When** a user reads paragraphs, **Then** text flows right-to-left naturally
3. **Given** Urdu docs contain lists, **When** displayed, **Then** bullet points appear on the right side of text
4. **Given** a user switches from Urdu to English, **When** the page loads, **Then** the HTML has `dir="ltr"` and `lang="en-US"` attributes

---

### Edge Cases

- **Untranslated Chapters**: If a chapter hasn't been translated yet, the Urdu URL `/ur/docs/...` shows English content (Docusaurus fallback behavior)
- **Homepage/Auth in Urdu**: Accessing `/ur/` or `/ur/auth` serves English content with Urdu URL (only docs are translated)
- **Direct URL Access**: Users can directly access `/ur/docs/...` URLs via shared links or bookmarks
- **SEO Considerations**: Each locale has separate sitemaps and proper `hreflang` tags for search engines
- **Mobile Devices**: Language dropdown remains accessible and functional on all screen sizes

## Requirements

### Functional Requirements

- **FR-001**: System MUST use Docusaurus native i18n with locale paths (`/ur/docs/...` for Urdu, `/docs/...` for English)
- **FR-002**: System MUST display language selector using built-in `localeDropdown` in the navbar
- **FR-003**: System MUST translate only documentation content (docs plugin), not homepage, auth, or chat pages
- **FR-004**: System MUST apply RTL text direction (`dir="rtl"`) for all Urdu pages
- **FR-005**: System MUST preserve code blocks, file paths, and technical syntax in English within Urdu docs
- **FR-006**: System MUST translate sidebar labels and chapter titles in the Urdu locale
- **FR-007**: System MUST NOT auto-redirect users to `/ur/` based on stored preferences
- **FR-008**: System MUST allow users to switch languages via navbar dropdown from any page
- **FR-009**: System MUST maintain user context when switching languages (same chapter in different locale)
- **FR-010**: System MUST serve pre-translated Urdu MDX files from `i18n/ur/docusaurus-plugin-content-docs/`
- **FR-011**: System MUST set appropriate HTML lang attributes (`lang="en-US"` or `lang="ur-PK"`)
- **FR-012**: System MUST configure Vercel to build and deploy all locales

### Non-Functional Requirements

- **NFR-001**: Language switching MUST occur within 2 seconds
- **NFR-002**: Build process MUST compile both English and Urdu locales in a single deployment
- **NFR-003**: Urdu URLs MUST be SEO-friendly and crawlable by search engines
- **NFR-004**: RTL layout MUST render consistently across Chrome, Firefox, Safari, and Edge
- **NFR-005**: Language dropdown MUST be accessible (keyboard navigable, screen reader compatible)

### Key Entities

- **Locale**: Language variant (en, ur) with associated content directory and configuration
- **Documentation Content**: MDX files organized in `docs/` (English) and `i18n/ur/docusaurus-plugin-content-docs/current/` (Urdu)
- **Sidebar Configuration**: Navigation structure defined in `current.json` for each locale
- **Locale Dropdown**: Built-in Docusaurus navbar component for language selection

## Success Criteria

### Measurable Outcomes

- **SC-001**: Users can switch between locales via navbar dropdown with < 2 second navigation time
- **SC-002**: Urdu docs display with correct RTL layout in 100% of translated chapters
- **SC-003**: Code blocks remain in English across all Urdu translations
- **SC-004**: Shared Urdu URLs (`/ur/docs/...`) load correctly for 100% of recipients
- **SC-005**: Language dropdown appears and functions on all page types (homepage, docs, auth)
- **SC-006**: Build process successfully generates both locales without errors
- **SC-007**: Sidebar labels and chapter titles display in Urdu for `/ur/docs/...` pages
- **SC-008**: No auto-redirect behavior - users control navigation explicitly

### Technical Validation

- Verify `docusaurus build` generates both `build/` (English) and `build/ur/` (Urdu)
- Verify Vercel deployment includes all locale directories
- Verify `hreflang` tags are present for SEO
- Verify RTL CSS is applied only to Urdu pages
- Verify no custom i18n components are needed (pure Docusaurus native)
