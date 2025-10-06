/*
 * Shared Dark Mode Functionality for C++ Crash Course Notes
 * Include this file in all HTML notes for consistent dark mode behavior
 */

(function() {
    'use strict';

    // Check for saved dark mode preference or default to light mode
    const savedMode = localStorage.getItem('darkMode');
    const prefersDark = window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches;
    const isDarkMode = savedMode === 'true' || (savedMode === null && prefersDark);

    // Apply dark mode on page load if needed
    function applyInitialMode() {
        if (isDarkMode && document.body) {
            document.body.classList.add('dark-mode');
        }
    }

    // Create and add toggle button
    function createToggleButton() {
        const button = document.createElement('button');
        button.className = 'dark-mode-toggle';
        button.setAttribute('aria-label', 'Toggle dark mode');
        button.setAttribute('title', 'Toggle dark mode');

        // Set initial icon
        updateButtonIcon(button, isDarkMode);

        // Add click handler
        button.addEventListener('click', toggleDarkMode);

        // Add to page
        document.body.appendChild(button);

        return button;
    }

    // Update button icon based on mode
    function updateButtonIcon(button, isDark) {
        button.textContent = isDark ? '☀️' : '🌙';
    }

    // Toggle dark mode
    function toggleDarkMode() {
        const body = document.body;
        const isDark = body.classList.toggle('dark-mode');

        // Save preference
        localStorage.setItem('darkMode', isDark);

        // Update button icon
        const button = document.querySelector('.dark-mode-toggle');
        if (button) {
            updateButtonIcon(button, isDark);
        }

        // Log for debugging
        console.log('Dark mode:', isDark ? 'enabled' : 'disabled');
    }

    // Initialize when DOM is ready
    function init() {
        applyInitialMode();
        createToggleButton();
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }

    // Optional: Listen for system preference changes
    if (window.matchMedia) {
        window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', function(e) {
            // Only auto-switch if user hasn't set a preference
            if (localStorage.getItem('darkMode') === null) {
                const shouldBeDark = e.matches;
                if (shouldBeDark !== document.body.classList.contains('dark-mode')) {
                    toggleDarkMode();
                }
            }
        });
    }

    // Expose toggle function globally for debugging
    window.toggleDarkMode = toggleDarkMode;
})();
