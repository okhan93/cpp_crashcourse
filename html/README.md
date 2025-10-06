# C++ Crash Course - HTML Notes

This folder contains HTML versions of the study notes for easy viewing in a web browser.

## Dark Mode

All HTML notes include a **shared dark mode** feature:

### Features
- 🌙 **Toggle button** in the top-right corner (moon icon for light mode, sun icon for dark mode)
- 💾 **Persistent preference** - your choice is saved in browser localStorage
- 🎨 **VSCode-inspired colors** for code syntax highlighting in dark mode
- 📱 **Smooth transitions** between light and dark modes
- 🖥️ **System preference detection** - automatically uses your OS dark mode preference on first visit

### How to Use

1. **Open any HTML file** in your browser:
   - `session1_notes.html`
   - `session2_notes.html`
   - `session3_notes.html`
   - `session4_notes.html`

2. **Click the toggle button** in the top-right corner to switch modes

3. **Your preference is saved** - when you open another session's notes, it will use the same mode

### Shared Files

The dark mode functionality is implemented using two shared files:

- **`dark-mode.css`** - Styles for dark mode (colors, syntax highlighting, etc.)
- **`dark-mode.js`** - JavaScript to handle toggling and persistence

All HTML files reference these shared files, so they have consistent dark mode behavior.

### Customization

To modify the dark mode colors, edit `dark-mode.css`. The CSS uses CSS variables for easy customization:

```css
:root {
    --bg-dark: #1e1e1e;
    --text-dark: #d4d4d4;
    --code-bg-dark: #2d2d2d;
    /* etc... */
}
```

## File Structure

```
html/
├── README.md              (this file)
├── dark-mode.css          (shared dark mode styles)
├── dark-mode.js           (shared dark mode functionality)
├── session1_notes.html    (Session 1: C++ Essentials)
├── session2_notes.html    (Session 2: OOP Basics)
├── session3_notes.html    (Session 3: Motion Tracking Fundamentals)
└── session4_notes.html    (Session 4: OptiTrack Simulation)
```

## Notes

- The HTML files are generated from Markdown using a VSCode extension
- Dark mode works offline (no internet connection required)
- Preference is stored per-browser (doesn't sync across devices)
