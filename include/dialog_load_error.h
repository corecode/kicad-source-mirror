#ifndef __dialog_load_error_h_
#define __dialog_load_error_h_

/**
@file
Subclass of DIALOG_DISPLAY_HTML_TEXT_BASE, which is generated by wxFormBuilder.
*/

#include "../common/dialogs/dialog_display_info_HTML_base.h"

/** Implementing DIALOG_LOAD_ERROR */
class DIALOG_LOAD_ERROR : public DIALOG_DISPLAY_HTML_TEXT_BASE
{
protected:
    // Handlers for DIALOG_LOAD_ERROR_BASE events.
    void OnCloseButtonClick( wxCommandEvent& event );

public:
    /** Constructor */
    DIALOG_LOAD_ERROR( wxWindow* parent );

    /**
     * Function ListSet
     * Add a list of items.
     * @param list = a string containing items. Items are separated by '\n'
     */
    void ListSet(const wxString &list);
    /**
     * Function ListSet
     * Add a list of items.
     * @param list = a wxArrayString containing items.
     */
    void ListSet(const wxArrayString &list);

    void ListClear();
    /**
     * Function MessageSet
     * Add a message (in bold) to message list.
     * @param message = the message
     */
    void MessageSet(const wxString &message);
};

#endif // __dialog_load_error_h_
