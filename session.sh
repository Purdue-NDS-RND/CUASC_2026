#!/bin/bash
SESSION="drone"

# Check the first argument passed to the script
case "$1" in
  init)
    # Start session in background if it doesn't exist
    tmux has-session -t $SESSION 2>/dev/null
    if [ $? != 0 ]; then
        tmux new-session -d -s $SESSION
        echo "🚀 Session '$SESSION' initialized in background."
    else
        echo "⚠️ Session '$SESSION' is already running."
    fi
    ;;

  join)
    # Attach to the session
    tmux attach-session -t $SESSION || echo "❌ No session found. Run './drone.sh init' first."
    ;;

  kill)
    # Stop everything
    tmux kill-session -t $SESSION && echo "🛑 Session '$SESSION' terminated."
    ;;

  *)
    # Show help if they type something else
    echo "Usage: ./drone.sh {init|join|kill}"
    ;;
esac
