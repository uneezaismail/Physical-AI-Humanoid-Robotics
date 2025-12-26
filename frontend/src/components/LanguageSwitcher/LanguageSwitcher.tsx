/**
 * LanguageSwitcher Component
 *
 * Interactive button for switching between English and Urdu content.
 * Positioned below chapter titles to enable quick translation access.
 *
 * Features:
 * - Toggles between English and Urdu using Docusaurus native i18n routing
 * - Navigates to equivalent page in other locale (/docs vs /ur/docs)
 * - Checks translation availability from build-time manifest
 * - Disables button with tooltip if translation unavailable
 * - Uses Docusaurus context for accurate current locale detection
 *
 * CRITICAL FIX: Now uses Docusaurus i18n.currentLocale instead of custom localStorage
 * to ensure button text matches the actual displayed page language.
 *
 * @see specs/002-urdu-translation/contracts/LanguageSwitcher.contract.ts for type contracts
 * @see specs/002-urdu-translation/quickstart.md Step 4 for usage guide
 */

import React from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import { useLanguagePreference } from '../../hooks/useLanguagePreference';
import translationManifest from '../../translation-manifest.json';
import type { LanguageSwitcherProps } from '../../types/i18n';
import { LANGUAGE_PREFERENCE_KEY } from '../../types/i18n';
import styles from './LanguageSwitcher.module.css';

/**
 * LanguageSwitcher Component
 *
 * @param {LanguageSwitcherProps} props - Component props
 * @returns Translation button
 */
export function LanguageSwitcher({
  className,
  iconOnly = false,
  onLanguageChange,
}: LanguageSwitcherProps): React.ReactElement {
  const { language, isLoading } = useLanguagePreference();
  const history = useHistory();
  const location = useLocation();

  /**
   * Determine if translation is available for current page
   * Extracts relative doc path from URL and checks manifest
   */
  const currentDocPath = React.useMemo(() => {
    // Remove locale prefix (/ur/) and /docs/ prefix to get relative path
    let path = location.pathname.replace(/^\/(ur\/)?docs\//, '');

    // Ensure path ends with .mdx (manifest keys use .mdx extension)
    if (!path.endsWith('.mdx') && !path.endsWith('.md')) {
      path += '.mdx';
    }

    return path;
  }, [location.pathname]);

  const translationAvailable = translationManifest[currentDocPath] === true;

  /**
   * Handle button click - switch language and navigate
   *
   * IMPORTANT: This uses Docusaurus router for proper i18n routing.
   * The route transformation:
   * - English -> Urdu: /docs/chapter-01 -> /ur/docs/chapter-01
   * - Urdu -> English: /ur/docs/chapter-01 -> /docs/chapter-01
   *
   * NEW (T024): Also persists language preference to localStorage for
   * cross-session persistence (User Story 2).
   */
  const handleClick = (): void => {
    const newLang = language === 'en' ? 'ur' : 'en';

    // Call callback if provided (for analytics, etc.)
    if (onLanguageChange) {
      onLanguageChange(newLang);
    }

    // T024: Persist language preference to localStorage (User Story 2)
    try {
      localStorage.setItem(LANGUAGE_PREFERENCE_KEY, newLang);
      console.log('[LanguageSwitcher] Language preference saved to localStorage:', newLang);
    } catch (error) {
      // T025: Handle localStorage unavailability (privacy mode, quota exceeded)
      console.warn('[LanguageSwitcher] Failed to save language preference to localStorage:', error);
      // Continue with navigation even if localStorage fails (graceful degradation)
    }

    // Navigate to equivalent page in other locale
    const currentPath = location.pathname;
    let newPath: string;

    if (newLang === 'ur') {
      // Switching to Urdu: add /ur prefix
      newPath = `/ur${currentPath}`;
    } else {
      // Switching to English: remove /ur prefix
      newPath = currentPath.replace(/^\/ur/, '') || '/';
    }

    // Navigate using Docusaurus router (this will trigger Docusaurus i18n routing)
    history.push(newPath + location.search);
  };

  /**
   * Determine button state and labels based on CURRENT page language
   * (not target language)
   */
  const isDisabled = !translationAvailable && language === 'en';

  // Button shows the TARGET language (opposite of current)
  const buttonText = language === 'en' ? 'Translate to Urdu' : 'Translate to English';
  const buttonIcon = language === 'en' ? 'اردو' : 'EN';

  const tooltip = isDisabled
    ? 'Urdu translation coming soon'
    : `Switch to ${language === 'en' ? 'Urdu' : 'English'}`;

  // Don't render during SSR or while loading
  if (isLoading) {
    return <div className={styles.languageSwitcher} aria-hidden="true" />;
  }

  return (
    <button
      className={`${styles.languageSwitcher} ${className || ''}`}
      onClick={handleClick}
      disabled={isDisabled}
      title={tooltip}
      aria-label={buttonText}
      type="button"
    >
      {iconOnly ? (
        <span className={styles.icon}>{buttonIcon}</span>
      ) : (
        <span className={styles.label}>{buttonText}</span>
      )}
    </button>
  );
}
