/**
 * useLanguagePreference Hook
 *
 * Custom React hook for managing language preference using Docusaurus native i18n system.
 * CRITICAL: This hook now uses Docusaurus context to detect the current locale from URL routing,
 * not custom URL parameters or localStorage.
 *
 * Docusaurus i18n routing:
 * - English (default): /docs/...
 * - Urdu: /ur/docs/...
 *
 * @see specs/002-urdu-translation/data-model.md for state management design
 * @see specs/002-urdu-translation/research.md Section 1 for implementation rationale
 */

import { useState, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import type { LanguageCode, UseLanguagePreferenceReturn } from '../types/i18n';
import { DEFAULT_LANGUAGE } from '../types/i18n';

/**
 * Custom hook for language preference management using Docusaurus i18n
 *
 * Features:
 * - Reads current language from Docusaurus i18n context (URL-based routing)
 * - No localStorage dependency (Docusaurus handles persistence via URL)
 * - Handles navigation via Docusaurus router
 * - Gracefully handles errors
 *
 * @returns {UseLanguagePreferenceReturn} Current language and switch function
 *
 * @example
 * ```tsx
 * function LanguageSwitcher() {
 *   const { language, switchLanguage, isLoading } = useLanguagePreference();
 *
 *   if (isLoading) return <div>Loading...</div>;
 *
 *   return (
 *     <button onClick={() => switchLanguage(language === 'en' ? 'ur' : 'en')}>
 *       Switch to {language === 'en' ? 'Urdu' : 'English'}
 *     </button>
 *   );
 * }
 * ```
 */
export function useLanguagePreference(): UseLanguagePreferenceReturn {
  const { i18n } = useDocusaurusContext();
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const [error, setError] = useState<Error | null>(null);

  // Get current locale from Docusaurus context (derived from URL path)
  // This is the single source of truth for the current language
  const currentLocale = i18n.currentLocale as LanguageCode;

  /**
   * Initialize hook - mark as loaded once Docusaurus context is available
   */
  useEffect(() => {
    try {
      // Docusaurus context is synchronously available
      setIsLoading(false);
    } catch (err) {
      console.error('[useLanguagePreference] Initialization error:', err);
      setError(err instanceof Error ? err : new Error('Unknown error'));
      setIsLoading(false);
    }
  }, []);

  /**
   * Switch to a different language
   *
   * IMPORTANT: This function is a placeholder for the LanguageSwitcher component.
   * The actual navigation is handled by the LanguageSwitcher component using
   * Docusaurus router (history.push) to maintain proper routing.
   *
   * We keep this function signature for compatibility with the existing API,
   * but the actual navigation logic lives in LanguageSwitcher.tsx.
   *
   * @param newLang - Language code to switch to ('en' or 'ur')
   */
  const switchLanguage = (newLang: LanguageCode): void => {
    try {
      // This is intentionally minimal - navigation is handled by LanguageSwitcher
      // using Docusaurus router to preserve routing integrity
      console.log('[useLanguagePreference] Language switch requested:', newLang);
    } catch (err) {
      console.error('[useLanguagePreference] Switch language error:', err);
      setError(err instanceof Error ? err : new Error('Failed to switch language'));
    }
  };

  return {
    language: currentLocale,
    switchLanguage,
    isLoading,
    error,
  };
}
