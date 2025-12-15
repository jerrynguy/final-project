import { useState, useCallback } from 'react';

/**
 * Hook for managing mission clarification flow
 * 
 * @returns {Object} Clarification state and methods
 */
export function useMissionClarification() {
  const [isWaitingForResponse, setIsWaitingForResponse] = useState(false);
  const [currentQuestion, setCurrentQuestion] = useState(null);
  const [clarificationPromise, setClarificationPromise] = useState(null);

  /**
   * Display clarification question and wait for user response
   * @param {Object} question - ClarificationQuestion object
   * @param {string} originalPrompt - Original user prompt
   * @returns {Promise<Object>} ClarificationResponse
   */
  const askClarification = useCallback((question, originalPrompt) => {
    setCurrentQuestion(question);
    setIsWaitingForResponse(true);

    return new Promise((resolve, reject) => {
      setClarificationPromise({ resolve, reject });
    });
  }, []);

  /**
   * Submit user's answer
   * @param {string} answer - User's response text
   */
  const submitAnswer = useCallback((answer) => {
    if (!clarificationPromise || !currentQuestion) return;

    const response = {
      original_prompt: '', // Will be filled by caller
      user_response: answer
    };

    clarificationPromise.resolve(response);
    
    // Reset state
    setCurrentQuestion(null);
    setIsWaitingForResponse(false);
    setClarificationPromise(null);
  }, [clarificationPromise, currentQuestion]);

  /**
   * Cancel clarification (abort mission)
   */
  const cancelClarification = useCallback(() => {
    if (!clarificationPromise) return;

    clarificationPromise.reject(new Error('Clarification cancelled by user'));
    
    // Reset state
    setCurrentQuestion(null);
    setIsWaitingForResponse(false);
    setClarificationPromise(null);
  }, [clarificationPromise]);

  return {
    isWaitingForResponse,
    currentQuestion,
    askClarification,
    submitAnswer,
    cancelClarification
  };
}