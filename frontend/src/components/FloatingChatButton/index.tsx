/**
 * FloatingChatButton - Floating chat button that appears on all pages
 */

import React, { useState } from "react";
import ChatWidget from "../ChatWidget";
import styles from "./styles.module.css";

const BotIcon = ({ size = 24, color = "currentColor" }) => (
  <svg
    width={size}
    height={size}
    viewBox="0 0 24 24"
    fill="none"
    stroke={color}
    strokeWidth="1.5"
    strokeLinecap="round"
    strokeLinejoin="round"
  >
    {/* Futuristic Robot Head */}
    <rect x="4" y="4" width="16" height="16" rx="4" />
    <line x1="4" y1="12" x2="20" y2="12" /> {/* Visor Line */}
    <line x1="8" y1="20" x2="8" y2="22" /> {/* Neck Left */}
    <line x1="16" y1="20" x2="16" y2="22" /> {/* Neck Right */}
    <circle cx="9" cy="9" r="1.5" fill={color} stroke="none" /> {/* Eye Left */}
    <circle cx="15" cy="9" r="1.5" fill={color} stroke="none" />{" "}
    {/* Eye Right */}
    <path d="M2 12h2" /> {/* Ear Left */}
    <path d="M20 12h2" /> {/* Ear Right */}
  </svg>
);

const FloatingChatButton: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Auto-open chat when text is selected (outside of chat widget)
  React.useEffect(() => {
    const handleTextSelection = () => {
      const selection = window.getSelection();
      if (!selection || selection.isCollapsed) return;

      const text = selection.toString().trim();
      if (!text || text.length < 3) return; // Ignore very short selections

      // Check if selection is inside the chat widget
      let node = selection.anchorNode;
      if (node && node.nodeType === 3) {
        node = node.parentNode;
      }

      // Check if selection is within chat components
      const chatElements = document.querySelectorAll(
        '[class*="chatContainer"], [class*="floatingChatWindow"]'
      );
      let isInChat = false;
      chatElements.forEach((chatEl) => {
        if (chatEl.contains(node as Node)) {
          isInChat = true;
        }
      });

      // If text is selected outside chat and chat is closed, open it
      if (!isInChat && !isOpen) {
        setIsOpen(true);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
    };
  }, [isOpen]);

  return (
    <>
      {/* Floating Chat Window */}
      {isOpen && ( // Use isChatOpen
        <div className={styles.floatingChatWindow}>
          <div className={styles.chatHeader}>
            <div className={styles.headerTitle}>
              <BotIcon size={24} color="#C6613F" />
              <h3>Assistant</h3>
            </div>
            <button className={styles.closeButton} onClick={toggleChat}>
              ×
            </button>
          </div>
          <div className={styles.chatContent}>
            <ChatWidget />
          </div>
        </div>
      )}

      {/* Floating Button */}
      {
        // Only show FAB if chat is not open
        <button
          className={styles.floatingButton}
          onClick={toggleChat}
          aria-label="Open AI chat assistant"
          title="Ask AI Assistant"
        >
          <BotIcon size={32} />
        </button>
      }
    </>
  );
};

export default FloatingChatButton;
