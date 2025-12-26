/**
 * Unit Tests for LanguageSwitcher Component
 *
 * Tests for User Story 1: Quick Chapter Translation
 *
 * Test Cases:
 * 1. Button renders with correct initial label ("Translate to Urdu" when English active)
 * 2. Button click toggles language state
 * 3. Button disabled when translation unavailable (mock manifest with chapter marked false)
 * 4. Button displays tooltip when disabled
 *
 * @see specs/002-urdu-translation/tasks.md T011
 * @see specs/002-urdu-translation/contracts/LanguageSwitcher.contract.ts
 */

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { LanguageSwitcher } from '../../src/components/LanguageSwitcher/LanguageSwitcher';
import { useLanguagePreference } from '../../src/hooks/useLanguagePreference';
import { useHistory, useLocation } from '@docusaurus/router';

// Mock dependencies
jest.mock('../../src/hooks/useLanguagePreference');
jest.mock('@docusaurus/router');
jest.mock('../../src/translation-manifest.json', () => ({
  'part-1-foundations-lab/chapter-01-embodied-ai.mdx': true,
  'part-1-foundations-lab/chapter-02-hardware-setup.mdx': false,
}));

const mockUseLanguagePreference = useLanguagePreference as jest.MockedFunction<typeof useLanguagePreference>;
const mockUseHistory = useHistory as jest.MockedFunction<typeof useHistory>;
const mockUseLocation = useLocation as jest.MockedFunction<typeof useLocation>;

describe('LanguageSwitcher Component', () => {
  let mockPush: jest.Mock;
  let mockSwitchLanguage: jest.Mock;

  beforeEach(() => {
    // Reset mocks before each test
    mockPush = jest.fn();
    mockSwitchLanguage = jest.fn();

    mockUseHistory.mockReturnValue({
      push: mockPush,
    } as any);
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  describe('Test Case 1: Initial Label Rendering', () => {
    it('should render "Translate to Urdu" when English is active', () => {
      // Arrange: Mock English page
      mockUseLanguagePreference.mockReturnValue({
        language: 'en',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/docs/part-1-foundations-lab/chapter-01-embodied-ai',
        search: '',
        hash: '',
      } as any);

      // Act: Render component
      render(<LanguageSwitcher />);

      // Assert: Button shows correct label
      const button = screen.getByRole('button');
      expect(button).toHaveTextContent('Translate to Urdu');
      expect(button).toHaveAttribute('aria-label', 'Translate to Urdu');
    });

    it('should render "Translate to English" when Urdu is active', () => {
      // Arrange: Mock Urdu page
      mockUseLanguagePreference.mockReturnValue({
        language: 'ur',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/ur/docs/part-1-foundations-lab/chapter-01-embodied-ai',
        search: '',
        hash: '',
      } as any);

      // Act: Render component
      render(<LanguageSwitcher />);

      // Assert: Button shows correct label
      const button = screen.getByRole('button');
      expect(button).toHaveTextContent('Translate to English');
      expect(button).toHaveAttribute('aria-label', 'Translate to English');
    });
  });

  describe('Test Case 2: Button Click Toggles Language', () => {
    it('should navigate to Urdu path when clicking from English page', () => {
      // Arrange: Mock English page
      mockUseLanguagePreference.mockReturnValue({
        language: 'en',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/docs/part-1-foundations-lab/chapter-01-embodied-ai',
        search: '',
        hash: '',
      } as any);

      render(<LanguageSwitcher />);

      // Act: Click button
      const button = screen.getByRole('button');
      fireEvent.click(button);

      // Assert: Navigation and language switch called
      expect(mockSwitchLanguage).toHaveBeenCalledWith('ur');
      expect(mockPush).toHaveBeenCalledWith('/ur/docs/part-1-foundations-lab/chapter-01-embodied-ai');
    });

    it('should navigate to English path when clicking from Urdu page', () => {
      // Arrange: Mock Urdu page
      mockUseLanguagePreference.mockReturnValue({
        language: 'ur',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/ur/docs/part-1-foundations-lab/chapter-01-embodied-ai',
        search: '',
        hash: '',
      } as any);

      render(<LanguageSwitcher />);

      // Act: Click button
      const button = screen.getByRole('button');
      fireEvent.click(button);

      // Assert: Navigation and language switch called
      expect(mockSwitchLanguage).toHaveBeenCalledWith('en');
      expect(mockPush).toHaveBeenCalledWith('/docs/part-1-foundations-lab/chapter-01-embodied-ai');
    });

    it('should preserve query parameters when navigating', () => {
      // Arrange: Mock English page with query params
      mockUseLanguagePreference.mockReturnValue({
        language: 'en',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/docs/part-1-foundations-lab/chapter-01-embodied-ai',
        search: '?highlight=sensors',
        hash: '',
      } as any);

      render(<LanguageSwitcher />);

      // Act: Click button
      const button = screen.getByRole('button');
      fireEvent.click(button);

      // Assert: Query params preserved
      expect(mockPush).toHaveBeenCalledWith('/ur/docs/part-1-foundations-lab/chapter-01-embodied-ai?highlight=sensors');
    });
  });

  describe('Test Case 3: Button Disabled When Translation Unavailable', () => {
    it('should disable button when translation not available', () => {
      // Arrange: Mock English page WITHOUT Urdu translation
      mockUseLanguagePreference.mockReturnValue({
        language: 'en',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/docs/part-1-foundations-lab/chapter-02-hardware-setup',
        search: '',
        hash: '',
      } as any);

      // Act: Render component
      render(<LanguageSwitcher />);

      // Assert: Button is disabled
      const button = screen.getByRole('button');
      expect(button).toBeDisabled();
    });

    it('should NOT disable button when on Urdu page (English always available)', () => {
      // Arrange: Mock Urdu page (English translation always available)
      mockUseLanguagePreference.mockReturnValue({
        language: 'ur',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/ur/docs/part-1-foundations-lab/chapter-02-hardware-setup',
        search: '',
        hash: '',
      } as any);

      // Act: Render component
      render(<LanguageSwitcher />);

      // Assert: Button is enabled (can always switch back to English)
      const button = screen.getByRole('button');
      expect(button).not.toBeDisabled();
    });

    it('should not call navigation when disabled button is clicked', () => {
      // Arrange: Mock English page WITHOUT Urdu translation
      mockUseLanguagePreference.mockReturnValue({
        language: 'en',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/docs/part-1-foundations-lab/chapter-02-hardware-setup',
        search: '',
        hash: '',
      } as any);

      render(<LanguageSwitcher />);

      // Act: Try to click disabled button
      const button = screen.getByRole('button');
      fireEvent.click(button);

      // Assert: No navigation happened
      expect(mockPush).not.toHaveBeenCalled();
      expect(mockSwitchLanguage).not.toHaveBeenCalled();
    });
  });

  describe('Test Case 4: Tooltip Display When Disabled', () => {
    it('should display "Urdu translation coming soon" tooltip when disabled', () => {
      // Arrange: Mock English page WITHOUT Urdu translation
      mockUseLanguagePreference.mockReturnValue({
        language: 'en',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/docs/part-1-foundations-lab/chapter-02-hardware-setup',
        search: '',
        hash: '',
      } as any);

      // Act: Render component
      render(<LanguageSwitcher />);

      // Assert: Button has correct title attribute
      const button = screen.getByRole('button');
      expect(button).toHaveAttribute('title', 'Urdu translation coming soon');
    });

    it('should display "Switch to Urdu" tooltip when enabled on English page', () => {
      // Arrange: Mock English page WITH Urdu translation
      mockUseLanguagePreference.mockReturnValue({
        language: 'en',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/docs/part-1-foundations-lab/chapter-01-embodied-ai',
        search: '',
        hash: '',
      } as any);

      // Act: Render component
      render(<LanguageSwitcher />);

      // Assert: Button has correct title attribute
      const button = screen.getByRole('button');
      expect(button).toHaveAttribute('title', 'Switch to Urdu');
    });

    it('should display "Switch to English" tooltip when on Urdu page', () => {
      // Arrange: Mock Urdu page
      mockUseLanguagePreference.mockReturnValue({
        language: 'ur',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/ur/docs/part-1-foundations-lab/chapter-01-embodied-ai',
        search: '',
        hash: '',
      } as any);

      // Act: Render component
      render(<LanguageSwitcher />);

      // Assert: Button has correct title attribute
      const button = screen.getByRole('button');
      expect(button).toHaveAttribute('title', 'Switch to English');
    });
  });

  describe('Additional Edge Cases', () => {
    it('should handle onLanguageChange callback if provided', () => {
      // Arrange: Mock with callback
      const mockCallback = jest.fn();

      mockUseLanguagePreference.mockReturnValue({
        language: 'en',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/docs/part-1-foundations-lab/chapter-01-embodied-ai',
        search: '',
        hash: '',
      } as any);

      render(<LanguageSwitcher onLanguageChange={mockCallback} />);

      // Act: Click button
      const button = screen.getByRole('button');
      fireEvent.click(button);

      // Assert: Callback invoked with new language
      expect(mockCallback).toHaveBeenCalledWith('ur');
    });

    it('should apply custom className if provided', () => {
      // Arrange
      mockUseLanguagePreference.mockReturnValue({
        language: 'en',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/docs/part-1-foundations-lab/chapter-01-embodied-ai',
        search: '',
        hash: '',
      } as any);

      // Act: Render with custom class
      const { container } = render(<LanguageSwitcher className="custom-class" />);

      // Assert: Custom class applied
      const button = container.querySelector('.custom-class');
      expect(button).toBeInTheDocument();
    });

    it('should render icon-only mode when iconOnly prop is true', () => {
      // Arrange
      mockUseLanguagePreference.mockReturnValue({
        language: 'en',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/docs/part-1-foundations-lab/chapter-01-embodied-ai',
        search: '',
        hash: '',
      } as any);

      // Act: Render with iconOnly
      render(<LanguageSwitcher iconOnly={true} />);

      // Assert: Button shows icon instead of text
      const button = screen.getByRole('button');
      expect(button).toHaveTextContent('UR'); // English page shows Urdu icon
      expect(button).not.toHaveTextContent('Translate to Urdu');
    });
  });

  describe('User Story 3: Visual Translation State Indication (T028)', () => {
    it('should have correct CSS class for styling', () => {
      // Arrange
      mockUseLanguagePreference.mockReturnValue({
        language: 'en',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/docs/part-1-foundations-lab/chapter-01-embodied-ai',
        search: '',
        hash: '',
      } as any);

      // Act: Render component
      const { container } = render(<LanguageSwitcher />);
      const button = container.querySelector('button');

      // Assert: Button has CSS module class for hover/disabled states
      expect(button).toHaveClass(/languageSwitcher/);
      expect(button).not.toBeNull();
    });

    it('should update aria-label when language changes', () => {
      // Arrange: Start with English
      mockUseLanguagePreference.mockReturnValue({
        language: 'en',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/docs/part-1-foundations-lab/chapter-01-embodied-ai',
        search: '',
        hash: '',
      } as any);

      const { rerender } = render(<LanguageSwitcher />);
      const buttonEn = screen.getByRole('button');

      // Assert: English page aria-label
      expect(buttonEn).toHaveAttribute('aria-label', 'Translate to Urdu');

      // Act: Switch to Urdu context
      mockUseLanguagePreference.mockReturnValue({
        language: 'ur',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/ur/docs/part-1-foundations-lab/chapter-01-embodied-ai',
        search: '',
        hash: '',
      } as any);

      rerender(<LanguageSwitcher />);
      const buttonUr = screen.getByRole('button');

      // Assert: Urdu page aria-label updated
      expect(buttonUr).toHaveAttribute('aria-label', 'Translate to English');
    });

    it('should show tooltip text via title attribute', () => {
      // Arrange: Enabled button
      mockUseLanguagePreference.mockReturnValue({
        language: 'en',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/docs/part-1-foundations-lab/chapter-01-embodied-ai',
        search: '',
        hash: '',
      } as any);

      // Act
      render(<LanguageSwitcher />);
      const button = screen.getByRole('button');

      // Assert: Tooltip provides action description
      expect(button).toHaveAttribute('title', 'Switch to Urdu');
      expect(button.getAttribute('title')).toMatch(/Switch to/);
    });

    it('should apply disabled styling when translation unavailable', () => {
      // Arrange: Chapter without Urdu translation
      mockUseLanguagePreference.mockReturnValue({
        language: 'en',
        switchLanguage: mockSwitchLanguage,
        isLoading: false,
        error: null,
      });

      mockUseLocation.mockReturnValue({
        pathname: '/docs/part-1-foundations-lab/chapter-02-hardware-setup',
        search: '',
        hash: '',
      } as any);

      // Act
      render(<LanguageSwitcher />);
      const button = screen.getByRole('button');

      // Assert: Disabled state
      expect(button).toBeDisabled();
      expect(button).toHaveAttribute('title', 'Urdu translation coming soon');
    });
  });
});
