import React, { useState } from 'react';
import { AlertCircle, X } from 'lucide-react';

/**
 * Modal for displaying mission clarification questions
 * 
 * @param {Object} props
 * @param {boolean} props.isOpen - Whether modal is visible
 * @param {Object} props.question - ClarificationQuestion object
 * @param {Function} props.onAnswer - Callback when user answers (answer: string) => void
 * @param {Function} props.onCancel - Callback when user cancels () => void
 */
export default function ClarificationModal({
  isOpen,
  question,
  onAnswer,
  onCancel
}) {
  const [customAnswer, setCustomAnswer] = useState('');
  
  if (!isOpen || !question) return null;

  const handleSubmit = () => {
    if (question.question_type === 'open_ended') {
      if (customAnswer.trim()) {
        onAnswer(customAnswer);
        setCustomAnswer('');
      }
    }
  };

  const getSeverityColor = () => {
    switch (question.ambiguity.severity) {
      case 'high': return 'text-red-400 border-red-500';
      case 'medium': return 'text-yellow-400 border-yellow-500';
      default: return 'text-blue-400 border-blue-500';
    }
  };

  return (
    <div className="fixed inset-0 bg-black/60 backdrop-blur-sm flex items-center justify-center z-50 p-4 animate-fade-in">
      <div className="bg-slate-800 rounded-2xl border border-slate-700 shadow-2xl max-w-lg w-full animate-scale-in">
        {/* Header */}
        <div className={`p-6 border-b border-slate-700 flex items-start gap-4 ${getSeverityColor()}`}>
          <AlertCircle className="w-6 h-6 mt-1 flex-shrink-0" />
          <div className="flex-1">
            <h3 className="text-xl font-bold text-white mb-1">
              Clarification Needed
            </h3>
            <p className="text-sm text-gray-400">
              {question.ambiguity.context}
            </p>
          </div>
          <button
            onClick={onCancel}
            className="text-gray-400 hover:text-white transition-colors"
          >
            <X className="w-5 h-5" />
          </button>
        </div>

        {/* Question */}
        <div className="p-6">
          <p className="text-lg text-white mb-6">
            {question.question_text}
          </p>

          {/* Choice Type */}
          {question.question_type === 'choice' && question.suggested_answers.length > 0 && (
            <div className="space-y-3">
              {question.suggested_answers.map((answer, index) => (
                <button
                  key={index}
                  onClick={() => onAnswer(answer)}
                  className="w-full text-left px-4 py-3 bg-slate-700/50 hover:bg-slate-600/50 border border-slate-600 rounded-lg transition-all transform hover:scale-102 hover:shadow-lg hover:shadow-purple-500/20"
                >
                  <span className="text-white">{answer}</span>
                </button>
              ))}
            </div>
          )}

          {/* Yes/No Type */}
          {question.question_type === 'yes_no' && (
            <div className="flex gap-3">
              <button
                onClick={() => onAnswer('yes')}
                className="flex-1 px-6 py-3 bg-green-600 hover:bg-green-500 text-white font-semibold rounded-lg transition-all transform hover:scale-105 shadow-lg shadow-green-500/30"
              >
                Yes
              </button>
              <button
                onClick={() => onAnswer('no')}
                className="flex-1 px-6 py-3 bg-red-600 hover:bg-red-500 text-white font-semibold rounded-lg transition-all transform hover:scale-105 shadow-lg shadow-red-500/30"
              >
                No
              </button>
            </div>
          )}

          {/* Open-Ended Type */}
          {question.question_type === 'open_ended' && (
            <div className="space-y-3">
              <input
                type="text"
                value={customAnswer}
                onChange={(e) => setCustomAnswer(e.target.value)}
                onKeyDown={(e) => e.key === 'Enter' && handleSubmit()}
                placeholder="Type your answer..."
                className="w-full px-4 py-3 bg-slate-700/50 border border-slate-600 rounded-lg text-white focus:ring-2 focus:ring-purple-500 focus:outline-none"
                autoFocus
              />
              <button
                onClick={handleSubmit}
                disabled={!customAnswer.trim()}
                className="w-full px-6 py-3 bg-purple-600 hover:bg-purple-500 text-white font-semibold rounded-lg transition-all transform hover:scale-105 shadow-lg shadow-purple-500/30 disabled:opacity-50 disabled:cursor-not-allowed disabled:transform-none"
              >
                Submit Answer
              </button>

              {/* Optional: Suggested answers as quick buttons */}
              {question.suggested_answers.length > 0 && (
                <div className="pt-3 border-t border-slate-700">
                  <p className="text-xs text-gray-400 mb-2">Quick suggestions:</p>
                  <div className="flex flex-wrap gap-2">
                    {question.suggested_answers.map((answer, index) => (
                      <button
                        key={index}
                        onClick={() => {
                          setCustomAnswer(answer);
                          onAnswer(answer);
                        }}
                        className="px-3 py-1 bg-slate-700/50 hover:bg-slate-600/50 text-sm text-white rounded-lg transition-all"
                      >
                        {answer}
                      </button>
                    ))}
                  </div>
                </div>
              )}
            </div>
          )}
        </div>

        {/* Footer */}
        <div className="px-6 py-4 bg-slate-900/50 rounded-b-2xl border-t border-slate-700 flex items-center justify-between">
          <div className="text-xs text-gray-400">
            <span className="font-semibold">{question.ambiguity.field}</span> - {question.ambiguity.type}
          </div>
          <button
            onClick={onCancel}
            className="text-sm text-gray-400 hover:text-white transition-colors"
          >
            Cancel Mission
          </button>
        </div>
      </div>

      <style jsx>{`
        @keyframes fade-in {
          from {
            opacity: 0;
          }
          to {
            opacity: 1;
          }
        }
        @keyframes scale-in {
          from {
            opacity: 0;
            transform: scale(0.95);
          }
          to {
            opacity: 1;
            transform: scale(1);
          }
        }
        .animate-fade-in {
          animation: fade-in 0.2s ease-out;
        }
        .animate-scale-in {
          animation: scale-in 0.3s ease-out;
        }
      `}</style>
    </div>
  );
}