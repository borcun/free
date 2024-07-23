Bookmarks
=========
> C-x r m RET : put a bookmark to current position
> C-x r b _bookmark_ RET : jump to _bookmark_
> C-x r l : list all bookmarks
> M-x bookmark-save : save all current bookmarks in the default file
                      which is ~/.emacs.d/bookmarks
> M-x bookmark-load RET _filename_ RET : load bookmarks from the file

Note that registers seem a little harder to understand than bookmarks.

Scrolling
=========
> C-v   : move next page
> M-v   : move previous page
> C-M-l : move current line top of the buffer, then change view
> C-l   : single C-l centerizes current line on the buffer, double C-l
          moves current line to top of buffer, triple C-l moves current
	  line to bottom of buffer

Narrowing
=========
> C-x n d : narrow active function region onto buffer temporarily
> C-x n n : narrow selected region onto buffer temporarily
> C-x n w : widen all buffer after narrowed

Text Scale
==========
> C-x C-+ : increase text height in current buffer
> C-x C-- : decrease text height in current buffer

Highlight Changes
=================
> M-x highlight-changes-mode : highlight changes in current buffer

Spell Check
===========
> M-$ : spell check for word under the cursor
> M-x flyspell-mode : active flyspell mode for active buffer

File Handling
=============
> C-x C-r : open file in read-only mode
> C-x 4 f : open file on next buffer
> C-x 5 f : find file for next buffer

Buffer Handling
===============
> C-x s   : save all buffers
> C-x C-w : save as current buffer
> C-x x g : revert buffer (M-x revert-buffer)
> C-x b   : select another buffer typing
> C-x 4 b : select another buffer typing, but for next window
> C-x 5 b : select a buffer on another emacs
> C-x C-q : toggle current buffer read-only
> C-x C-b : show all buffers (%: read-only, *:modified, .: current)
             d: sign buffer to delete,
	     s: sign buffer to save
	     x: execute delete and save operations
	     u: undo delete and save operations
> C-x 4 c : copy, then paste current buffer on next window
> C-M-v   : scroll next buffer upward
> C-M-S-v : scroll next buffer downward
> C-M-S-l : recenter next buffer
> C-x -   : shrink current buffer vertically if including many lines
> C-x +   : make all windows the same height (balance-windows)
> C-x C-p : select whole buffer
> C-x l   : count whole lines or the buffer
> C-x C-l : convert region to lowercase
> C-x C-U : convert region to uppercase

Timestamp
=========
> When you write 'Time-stamp: <>' or 'Time-stamp: " "' in first eight
  lines of the file, then M-x time-stamp on echo area, it puts current
  time stamp between < and > or " and " symbols. If you put time-stamp
  function to the hook before-save-hook, it is updated when the file
  is saved.

Diff Mode
=========
> M-x diff      : active diff mode, then select two files to compare
> M-x diff-mode : enable diff-mode, highlight diff between two files

Indentation
===========
> M-i   : move current line as tab
> C-M-\ : indent whole buffer
> M-x align : align whole lines

Sentences
=========
> M-a : move back to the beginning of the sentence
> M-e : move forward to the beginning of the sentence
> M-h : put point and mark around this or next paragraph

Mail
====
> C-x m : start composing a mail message

Bars
====
> M-x menu-bar-mode : enable/disable menu bar
> M-x tool-bar-mode : enable/disable tool bar
> M-x tab-bar-mode  : enable/disable tab bar