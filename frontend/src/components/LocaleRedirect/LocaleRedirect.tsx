/**
 * LocaleRedirect Component
 *
 * Handles automatic redirection to user's preferred locale on initial page load.
 * Implements User Story 2 (US2): Persistent Language Preference
 *
 * Behavior:
 * - On root/default locale pages, checks localStorage for language preference
 * - If user prefers Urdu ('ur') but is on English page, redirects to Urdu version
 * - If user prefers English ('en') or no preference exists, stays on current page
 * - Handles localStorage unavailability gracefully (privacy mode)
 * - Does NOT redirect if user explicitly navigated to a specific locale URL
 *
 * Integration:
 * - Must be rendered once at app root level (in Root component or theme wrapper)
 * - Runs on every route change to handle deep links and navigation
 *
 * @see specs/002-urdu-translation/tasks.md T023
 * @see specs/002-urdu-translation/spec.md User Story 2
 */

import { useEffect, useRef } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import { LANGUAGE_PREFERENCE_KEY } from '../../types/i18n';
import type { LanguageCode } from '../../types/i18n';

/**
 * LocaleRedirect Component
 *
 * Invisible component that handles automatic locale redirection based on
 * user's stored language preference.
 *
 * @returns null (renders nothing)
 */
export function LocaleRedirect(): null {
  const history = useHistory();
  const location = useLocation();
  const hasRedirected = useRef(false);

  useEffect(() => {
    // Skip if already redirected in this session
    if (hasRedirected.current) {
      return;
    }

    // Only check preference on default (English) locale pages
    const isDefaultLocale = !location.pathname.startsWith('/ur/');

    if (!isDefaultLocale) {
      // User is already on Urdu page, no redirect needed
      return;
    }

    try {
      // Read language preference from localStorage
      const storedPreference = localStorage.getItem(LANGUAGE_PREFERENCE_KEY);

      if (storedPreference === 'ur') {
        // User prefers Urdu but is on English page - redirect
        const urduPath = `/ur${location.pathname}${location.search}${location.hash}`;

        console.log('[LocaleRedirect] Redirecting to preferred Urdu locale:', urduPath);

        // Use replace instead of push to avoid back button issues
        history.replace(urduPath);

        // Mark as redirected to prevent redirect loops
        hasRedirected.current = true;
      } else if (storedPreference === 'en' || storedPreference === null) {
        // User prefers English or has no preference - stay on English
        console.log('[LocaleRedirect] Staying on default English locale');
      } else {
        // Invalid value in localStorage - ignore
        console.warn('[LocaleRedirect] Invalid language preference in localStorage:', storedPreference);
      }
    } catch (error) {
      // localStorage unavailable (privacy mode, quota exceeded, etc.)
      // Gracefully degrade - stay on current page
      console.warn('[LocaleRedirect] localStorage unavailable, staying on current locale:', error);
    }
  }, [location.pathname, location.search, location.hash, history]);

  // This component renders nothing
  return null;
}

/**
 * Hook version of LocaleRedirect for use in functional components
 *
 * Usage:
 * ```tsx
 * function MyComponent() {
 *   useLocaleRedirect();
 *   return <div>Content</div>;
 * }
 * ```
 */
export function useLocaleRedirect(): void {
  const history = useHistory();
  const location = useLocation();
  const hasRedirected = useRef(false);

  useEffect(() => {
    if (hasRedirected.current) return;

    const isDefaultLocale = !location.pathname.startsWith('/ur/');
    if (!isDefaultLocale) return;

    try {
      const storedPreference = localStorage.getItem(LANGUAGE_PREFERENCE_KEY) as LanguageCode | null;

      if (storedPreference === 'ur') {
        const urduPath = `/ur${location.pathname}${location.search}${location.hash}`;
        history.replace(urduPath);
        hasRedirected.current = true;
      }
    } catch (error) {
      console.warn('[useLocaleRedirect] localStorage unavailable:', error);
    }
  }, [location.pathname, location.search, location.hash, history]);
}
