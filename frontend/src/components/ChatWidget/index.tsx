/**
 * ChatWidget - Custom AI-powered textbook assistant.
 *
 * Features:
 * - Ask questions about Physical AI & Robotics
 * - Get RAG-grounded answers with citations
 * - Simple, clean chat interface
 */

import React, { useState, useRef, useEffect } from 'react';
import { apiClient, ChatResponse } from '../../lib/api';
import styles from './styles.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: ChatResponse['sources'];
}

const ChatWidget: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: 'welcome',
      role: 'assistant',
      content: '👋 Hi! I\'m your Physical AI textbook assistant. Ask me anything about ROS 2, robotics, URDF, or any topic from the book!',
      timestamp: new Date(),
    },
  ]);

  const [inputValue, setInputValue] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const chatContainerRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      if (!selection || selection.isCollapsed) return;

      const text = selection.toString().trim();
      if (text) {
        // Check if selection is inside the chat widget
        let node = selection.anchorNode;
        // If anchorNode is text node, use its parent
        if (node && node.nodeType === 3) {
          node = node.parentNode;
        }
        
        if (
          chatContainerRef.current &&
          node &&
          chatContainerRef.current.contains(node)
        ) {
          return;
        }
        setSelectedText(text);
      }
    };

    // Check for initial selection on mount
    handleMouseUp();

    document.addEventListener('mouseup', handleMouseUp);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

  const handleSendMessage = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) return;

    // Construct display content
    let displayContent = inputValue;
    if (selectedText) {
      displayContent = `**Context:**\n> ${selectedText}\n\n${inputValue}`;
    }

    const userMessage: Message = {
      id: `user-${Date.now()}`,
      role: 'user',
      content: displayContent,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    
    const query = inputValue;
    const currentSelectedText = selectedText;
    
    setInputValue('');
    setSelectedText('');
    setIsLoading(true);

    try {
      const response = await apiClient.sendChatMessage(query, currentSelectedText);

      const assistantMessage: Message = {
        id: `assistant-${Date.now()}`,
        role: 'assistant',
        content: response.answer,
        timestamp: new Date(),
        sources: response.sources,
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (error) {
      const errorMessage: Message = {
        id: `error-${Date.now()}`,
        role: 'assistant',
        content: `❌ Error: ${error instanceof Error ? error.message : 'Unknown error'}.`,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.chatContainer} ref={chatContainerRef}>
      <div className={styles.messagesContainer}>
        {messages.map((message) => (
          <div
            key={message.id}
            className={`${styles.message} ${
              message.role === 'user' ? styles.userMessage : styles.assistantMessage
            }`}
          >
            <div className={styles.messageContent}>
              <div style={{ whiteSpace: 'pre-wrap' }}>{message.content}</div>
              {message.sources && message.sources.length > 0 && (
                <div className={styles.sources}>
                  <strong>📚 Sources:</strong>
                  <ul>
                    {message.sources.map((source, idx) => (
                      <li key={idx}>
                        {source.title} - {source.section} ({(source.score * 100).toFixed(0)}% relevant)
                      </li>
                    ))}
                  </ul>
                </div>
              )}
            </div>
          </div>
        ))}
        {isLoading && (
          <div className={`${styles.message} ${styles.assistantMessage}`}>
            <div className={styles.loadingDots}>
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <div className={styles.inputArea}>
        {selectedText && (
          <div className={styles.selectedTextPreview}>
            <div className={styles.selectedTextContent}>
              <small>Context:</small>
              <div>"{selectedText.length > 60 ? selectedText.substring(0, 60) + '...' : selectedText}"</div>
            </div>
            <button 
              type="button" 
              className={styles.clearSelectionButton}
              onClick={() => setSelectedText('')}
            >
              ×
            </button>
          </div>
        )}
        <form className={styles.inputForm} onSubmit={handleSendMessage}>
          <input
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            placeholder="Ask about ROS 2, URDF, sensors..."
            className={styles.input}
            disabled={isLoading}
          />
          <button
            type="submit"
            className={styles.sendButton}
            disabled={isLoading || !inputValue.trim()}
            aria-label="Send message"
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
              <line x1="12" y1="19" x2="12" y2="5"></line>
              <polyline points="5 12 12 5 19 12"></polyline>
            </svg>
          </button>
        </form>
      </div>
    </div>
  );
};

export default ChatWidget;
