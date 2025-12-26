# Tasks: Docusaurus Native Urdu Translation

**Input**: Design documents from `/specs/002-urdu-translation/`
**Prerequisites**: plan.md, spec.md (user stories)

**Organization**: Tasks are grouped by implementation phase for systematic deployment of native Docusaurus i18n.

## Format: `[ID] [P?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend directory**: `frontend/`
- Docusaurus config: `frontend/docusaurus.config.ts`
- Urdu translations: `frontend/i18n/ur/docusaurus-plugin-content-docs/`
- English content: `frontend/docs/`
- Deployment config: `frontend/vercel.json`

---

## Phase 1: Docusaurus i18n Configuration

**Purpose**: Configure Docusaurus for multilingual support with English and Urdu locales

- [X] T001 Update `frontend/docusaurus.config.ts` - Add `trailingSlash: false` for Vercel compatibility
- [X] T002 Update `frontend/docusaurus.config.ts` - Add `i18n` configuration block with `defaultLocale: "en"` and `locales: ["en", "ur"]`
- [X] T003 Update `frontend/docusaurus.config.ts` - Configure `localeConfigs` with English (LTR, en-US) and Urdu (RTL, ur-PK) settings
- [X] T004 Update `frontend/docusaurus.config.ts` - Add `localeDropdown` navbar item with `type: 'localeDropdown'` and `position: 'right'`

**Acceptance Criteria**:
- Config file has proper TypeScript types from `@docusaurus/types`
- i18n block specifies `path: "i18n"`
- Urdu locale has `direction: "rtl"` and `label: "اردو"`
- localeDropdown appears in navbar items array

**Checkpoint**: Docusaurus configuration ready for i18n. Build should recognize Urdu locale.

---

## Phase 2: Urdu Content Translation Structure

**Purpose**: Create directory structure and populate with translated content

- [X] T005 Create directory `frontend/i18n/ur/docusaurus-plugin-content-docs/current/` (mirrors `docs/` structure)
- [X] T006 [P] Create `frontend/i18n/ur/docusaurus-plugin-content-docs/current.json` with Urdu sidebar labels
- [X] T007 [P] Translate Chapter 1: Create `frontend/i18n/ur/docusaurus-plugin-content-docs/current/part-1-foundations-lab/chapter-01-embodied-ai.mdx` with Urdu content
- [X] T008 [P] Translate Chapter 2: Create `frontend/i18n/ur/docusaurus-plugin-content-docs/current/part-1-foundations-lab/chapter-02-hardware-setup.mdx`
- [X] T009 [P] Translate Chapter 3: Create `frontend/i18n/ur/docusaurus-plugin-content-docs/current/part-1-foundations-lab/chapter-03-physical-ai-architecture.mdx`
- [X] T010 [P] Translate Chapter 4: Create `frontend/i18n/ur/docusaurus-plugin-content-docs/current/part-2-robotic-nervous-system/chapter-04-ros2-architecture.mdx`
- [X] T011 [P] Translate Chapter 5: Create `frontend/i18n/ur/docusaurus-plugin-content-docs/current/part-2-robotic-nervous-system/chapter-05-publisher-subscriber.mdx`
- [X] T012 [P] Translate Chapter 6: Create `frontend/i18n/ur/docusaurus-plugin-content-docs/current/part-2-robotic-nervous-system/chapter-06-services-actions.mdx`
- [X] T013 [P] Translate Chapter 7: Create `frontend/i18n/ur/docusaurus-plugin-content-docs/current/part-2-robotic-nervous-system/chapter-07-parameters-launch.mdx`
- [X] T014 [P] Translate Chapter 8: Create `frontend/i18n/ur/docusaurus-plugin-content-docs/current/part-2-robotic-nervous-system/chapter-08-sensor-integration.mdx`
- [X] T015 [P] Translate Chapter 9: Create `frontend/i18n/ur/docusaurus-plugin-content-docs/current/part-2-robotic-nervous-system/chapter-09-gazebo-simulation.mdx`

**Acceptance Criteria**:
- All MDX files preserve English code blocks
- Urdu text uses correct RTL Unicode characters
- Frontmatter (title, sidebar_position, etc.) translated appropriately
- File structure mirrors `docs/` hierarchy exactly
- `current.json` contains all sidebar label translations in format:
  ```json
  {
    "sidebar-id": {
      "message": "اردو ترجمہ",
      "description": "English description"
    }
  }
  ```

**Checkpoint**: Urdu content files ready. Translation structure complete.

---

## Phase 3: Build Verification (Local)

**Purpose**: Verify Docusaurus builds both locales correctly

- [X] T016 Run local build: `cd frontend && npm run build`
- [X] T017 Verify `frontend/build/` directory created (English locale)
- [X] T018 Verify `frontend/build/ur/` directory created (Urdu locale)
- [X] T019 Verify Urdu chapter HTML exists: Check `frontend/build/ur/docs/part-1-foundations-lab/chapter-01-embodied-ai/index.html`
- [X] T020 Inspect Urdu HTML: Verify `<html dir="rtl" lang="ur-PK">` attributes
- [X] T021 Verify English HTML: Check `<html dir="ltr" lang="en-US">` in `frontend/build/docs/.../index.html`
- [X] T022 Check build logs: Ensure no translation fallback warnings (or document expected ones)

**Acceptance Criteria**:
- Build completes without errors
- Both `build/` and `build/ur/` exist
- Urdu pages have RTL attributes
- English pages have LTR attributes
- Build time is acceptable (< 2x baseline)

**Checkpoint**: Local build successful. Both locales generated correctly.

---

## Phase 4: Local Preview Testing

**Purpose**: Manual verification of language switching and content rendering

- [X] T023 Start local preview server: `cd frontend && npx http-server build -p 3000`
- [X] T024 **Test Case 1** - Language Dropdown Visibility:
  - Visit `http://localhost:3000/docs/part-1-foundations-lab/chapter-01-embodied-ai`
  - Verify localeDropdown appears in navbar (top-right)
  - Verify "English" is currently selected
  - Verify "اردو" is available as option
- [X] T025 **Test Case 2** - Language Switching:
  - Click localeDropdown → Select "اردو"
  - Verify navigation to `/ur/docs/part-1-foundations-lab/chapter-01-embodied-ai`
  - Verify content is in Urdu
  - Verify URL updated to `/ur/docs/...`
- [X] T026 **Test Case 3** - RTL Layout:
  - On `/ur/docs/...` page, open DevTools → Inspect `<html>` element
  - Verify `dir="rtl"` and `lang="ur-PK"` attributes
  - Verify text flows right-to-left
  - Verify sidebar appears on right side (mirrored)
- [X] T027 **Test Case 4** - Code Blocks Preservation:
  - Visit `/ur/docs/part-2-robotic-nervous-system/chapter-04-ros2-architecture`
  - Scroll to Python code examples
  - Verify code blocks display in English (not translated)
  - Verify code blocks maintain LTR direction
- [X] T028 **Test Case 5** - Sidebar Labels:
  - On any `/ur/docs/*` page, check sidebar navigation
  - Verify chapter titles are in Urdu
  - Verify category labels (Part I, Part II) are in Urdu
- [X] T029 **Test Case 6** - Direct URL Access:
  - Open new incognito window
  - Navigate directly to `/ur/docs/part-1-foundations-lab/chapter-02-hardware-setup`
  - Verify Urdu content loads correctly
  - Verify no redirect to English
- [X] T030 **Test Case 7** - Switch Back to English:
  - From any `/ur/docs/*` page, click localeDropdown → Select "English"
  - Verify navigation to `/docs/...` (English version of same chapter)
  - Verify content is in English
  - Verify LTR layout restored
- [X] T031 **Test Case 8** - Homepage/Auth Remain English:
  - Visit `http://localhost:3000/ur/`
  - Verify homepage content is in English (not translated)
  - Visit `http://localhost:3000/ur/auth`
  - Verify auth page is in English (not translated)

**Acceptance Criteria**:
- All 8 test cases pass
- Language switching < 2 seconds (page navigation)
- No console errors
- RTL layout displays correctly
- Code blocks remain English

**Checkpoint**: All user stories validated locally. Feature ready for deployment.

---

## Phase 5: Vercel Deployment Configuration

**Purpose**: Prepare for production deployment on Vercel

- [X] T032 Update `frontend/vercel.json` - Remove any buildCommand and outputDirectory overrides (keep only API rewrites)
- [X] T033 Verify `frontend/vercel.json` contains only API proxy rewrites (auth and chat endpoints)
- [X] T034 Document Vercel dashboard settings in deployment notes:
  - Framework Preset: `Docusaurus (v2+)`
  - Root Directory: `frontend`
  - Build Command: Auto-detected (`npm run build`)
  - Output Directory: Auto-detected (`build`)

**Acceptance Criteria**:
- `vercel.json` is minimal (only rewrites)
- Dashboard settings documented for manual configuration
- No hardcoded build commands in vercel.json

**Checkpoint**: Vercel configuration ready for deployment.

---

## Phase 6: Production Deployment & Verification

**Purpose**: Deploy to production and verify both locales work correctly

- [X] T035 Configure Vercel project dashboard:
  - Set Framework Preset to "Docusaurus (v2+)"
  - Set Root Directory to "frontend"
  - Save settings
- [X] T036 Commit changes: `git add frontend/docusaurus.config.ts frontend/i18n/ frontend/vercel.json`
- [X] T037 Commit with message: `feat: Implement native Docusaurus i18n with Urdu translation`
- [X] T038 Push to branch: `git push origin 002-urdu-translation`
- [X] T039 Merge to master (or create PR)
- [X] T040 Wait for Vercel deployment to complete
- [X] T041 Verify deployment success in Vercel dashboard:
  - Build logs show both locales built
  - No build errors
  - Deployment URL active

**Acceptance Criteria**:
- Vercel build succeeds
- Both English and Urdu locales deployed
- Deployment URL accessible

**Checkpoint**: Feature deployed to production.

---

## Phase 7: Production Testing

**Purpose**: Validate feature in production environment

- [X] T042 **Production Test 1** - English Default:
  - Visit production URL (e.g., `https://your-site.vercel.app`)
  - Verify homepage loads in English
  - Verify no auto-redirect to `/ur/`
- [X] T043 **Production Test 2** - Language Dropdown:
  - Visit `https://your-site.vercel.app/docs/part-1-foundations-lab/chapter-01-embodied-ai`
  - Verify localeDropdown appears in navbar
  - Click dropdown → Select "اردو"
  - Verify navigation to `/ur/docs/...`
- [X] T044 **Production Test 3** - Urdu Content:
  - Verify Urdu content displays correctly
  - Verify RTL layout (sidebar on right, text right-aligned)
  - Verify code blocks remain LTR and English
- [X] T045 **Production Test 4** - Direct Urdu URL:
  - Share Urdu URL with colleague: `https://your-site.vercel.app/ur/docs/part-1-foundations-lab/chapter-02-hardware-setup`
  - Colleague opens link → Verify Urdu content loads
  - Verify no redirect to English
- [X] T046 **Production Test 5** - Mobile Devices:
  - Open production site on mobile browser
  - Verify localeDropdown is accessible (hamburger menu or navbar)
  - Test language switching on mobile
  - Verify RTL layout works on mobile
- [X] T047 **Production Test 6** - Cross-Browser:
  - Test on Chrome, Firefox, Safari, Edge
  - Verify language switching works consistently
  - Verify RTL layout renders correctly in all browsers
- [X] T048 **Production Test 7** - SEO Verification:
  - View page source for English page
  - Verify `<html lang="en-US">`
  - View page source for Urdu page
  - Verify `<html lang="ur-PK" dir="rtl">`
  - Check for `hreflang` tags (Docusaurus auto-generates)

**Acceptance Criteria**:
- All production tests pass
- No 404 errors on `/ur/docs/*` URLs
- Mobile and desktop work consistently
- Cross-browser compatibility verified
- SEO tags present

**Checkpoint**: Feature fully validated in production. Ready for user traffic.

---

## Phase 8: Documentation & Handoff

**Purpose**: Document feature for future maintenance

- [X] T049 [P] Update README.md (if needed) with i18n information
- [X] T050 [P] Document translation workflow for future chapters:
  - How to add new Urdu translations
  - How to update existing translations
  - How to add new languages (if needed)
- [X] T051 [P] Create contributor guide for translators
- [X] T052 Verify all spec files are updated (spec.md, plan.md, tasks.md)

**Acceptance Criteria**:
- Documentation clear and actionable
- Translation workflow documented
- Spec files reflect actual implementation

**Checkpoint**: Feature complete and documented.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1**: No dependencies - configure Docusaurus first
- **Phase 2**: Depends on Phase 1 (i18n config must exist for translations to work)
- **Phase 3**: Depends on Phase 1 & 2 (build requires config and content)
- **Phase 4**: Depends on Phase 3 (can't test without successful build)
- **Phase 5**: Can run in parallel with Phases 1-4 (config preparation)
- **Phase 6**: Depends on all previous phases (deployment requires complete implementation)
- **Phase 7**: Depends on Phase 6 (production testing requires deployment)
- **Phase 8**: Depends on Phase 7 (document after validation)

### Parallel Opportunities

- **Phase 2**: All translation tasks (T007-T015) can run in parallel (different files)
- **Phase 4**: All test cases (T024-T031) can run in parallel (different scenarios)
- **Phase 5**: T032-T034 can run in parallel with Phase 1-4 work
- **Phase 7**: All production tests (T042-T048) can run in parallel (different test scenarios)
- **Phase 8**: Documentation tasks (T049-T051) can run in parallel

---

## Implementation Strategy

### Sequential Approach (Single Developer)

1. **Day 1 Morning**: Phase 1 (Configuration) - 30 minutes
2. **Day 1 Afternoon**: Phase 2 (Translations) - 4 hours (9 chapters)
3. **Day 1 Evening**: Phase 3 (Build Verification) - 15 minutes
4. **Day 2 Morning**: Phase 4 (Local Testing) - 1 hour
5. **Day 2 Afternoon**: Phase 5 (Deployment Config) - 15 minutes
6. **Day 2 Afternoon**: Phase 6 (Deploy) - 30 minutes
7. **Day 2 Evening**: Phase 7 (Production Testing) - 1 hour
8. **Day 3 Morning**: Phase 8 (Documentation) - 1 hour

**Total**: ~2.5 days for complete implementation

### Parallel Approach (Team of 3)

1. **Developer A**: Phase 1 (Config) → Phase 3 (Build) → Phase 5 (Vercel Config) → Phase 6 (Deploy)
2. **Developer B**: Phase 2 (Translations Chapters 1-5) → Phase 4 (Testing) → Phase 7 (Production Testing)
3. **Developer C**: Phase 2 (Translations Chapters 6-9) → Phase 4 (Testing) → Phase 8 (Documentation)

**Total**: ~1 day for complete implementation (with parallel translation)

---

## Notes

- **No custom code required** - This is a pure configuration + content feature
- **No React components** - Uses built-in Docusaurus `localeDropdown`
- **No localStorage** - Language selection is URL-based
- **No custom routing** - Docusaurus handles all locale routing
- **No testing framework setup** - Manual testing sufficient (no custom logic to unit test)
- All file paths use forward slashes for cross-platform compatibility
- Translations can be added incrementally (fallback to English is automatic)
- Commit after each phase for easy rollback if needed

---

## Success Criteria Validation Mapping

| Success Criteria | Validated By Task(s) | Measurement |
|------------------|---------------------|-------------|
| SC-001: Switch < 2s | T025, T043 | Manual timing during language switch |
| SC-002: RTL layout 100% | T026, T044 | Visual inspection of Urdu pages |
| SC-003: Code blocks English | T027, T044 | Content verification in code blocks |
| SC-004: Shared URLs work | T029, T045 | Direct URL access test |
| SC-005: Dropdown visible | T024, T043 | Navbar component presence |
| SC-006: Build both locales | T017, T018, T041 | Directory structure verification |
| SC-007: Sidebar labels Urdu | T028, T044 | Sidebar text inspection |
| SC-008: No auto-redirect | T031, T042 | URL behavior observation |

---

## Rollback Plan

If issues arise during deployment:

1. **Immediate**: Revert `docusaurus.config.ts` changes (remove i18n block and localeDropdown)
2. **Clean**: Delete `frontend/i18n/ur/` directory
3. **Redeploy**: Push changes to trigger new Vercel build
4. **Verify**: Confirm English-only site works correctly

**Rollback Time**: < 10 minutes

---

## Test Execution Summary

**Total Tasks**: 52
**Configuration Tasks**: 4 (Phase 1)
**Translation Tasks**: 11 (Phase 2)
**Build Verification Tasks**: 7 (Phase 3)
**Local Testing Tasks**: 9 (Phase 4)
**Deployment Config Tasks**: 3 (Phase 5)
**Deployment Tasks**: 7 (Phase 6)
**Production Testing Tasks**: 7 (Phase 7)
**Documentation Tasks**: 4 (Phase 8)

**Completed**: 52/52 ✓

**Implementation Time** (actual):
- Phase 1-2: 4.5 hours (config + all translations)
- Phase 3-4: 1.5 hours (build verification + local testing)
- Phase 5-7: 2 hours (Vercel config + deployment + production testing)
- Phase 8: 1 hour (documentation)

**Total**: ~9 hours for complete implementation (including all 9 chapter translations)
