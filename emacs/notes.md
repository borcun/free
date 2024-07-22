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
> C-v and M-v : move next page and previous page respectively
> C-M-l : move current line top of the buffer, then change view
> C-l : single C-l centerizes current line on the buffer, double C-l
        moves current line to top of buffer, triple C-l moves current
		line to bottom of buffer

Narrowing
=========
> C-x n n : narrow selected region onto buffer temporarily
> C-x n w : widen all buffer after narrowed

Text Scale
==========
> C-x C-+ : increase text height in current buffer
> C-x C-- : decrease text height in current buffer

Highlight Changes
=================
> M-x highlight-changes-mode : highlight changes in current buffer
