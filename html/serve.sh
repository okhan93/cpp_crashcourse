#!/bin/bash
# Simple HTTP server to view HTML notes with dark mode
# This allows the browser to load external CSS and JS files

echo "Starting HTTP server for C++ Crash Course notes..."
echo "Open your browser to: http://localhost:8000"
echo ""
echo "Available files:"
echo "  http://localhost:8000/session1_notes.html"
echo "  http://localhost:8000/session2_notes.html"
echo "  http://localhost:8000/session3_notes.html"
echo "  http://localhost:8000/session4_notes.html"
echo ""
echo "Press Ctrl+C to stop the server"
echo ""

# Start Python HTTP server (works with Python 2 or 3)
if command -v python3 &> /dev/null; then
    python3 -m http.server 8000
elif command -v python &> /dev/null; then
    python -m SimpleHTTPServer 8000
else
    echo "Error: Python not found. Please install Python to run the server."
    exit 1
fi
