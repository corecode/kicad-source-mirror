/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2024-2026 KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 * or you may search the http://www.gnu.org website for the version 2 license,
 * or you may write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#ifndef SPICE_MODEL_DOWNLOADER_H
#define SPICE_MODEL_DOWNLOADER_H

#include <wx/string.h>
#include <vector>


struct SPICE_MODEL_RESULT
{
    bool     success;
    wxString spiceText;  ///< Full .SUBCKT text (for inline use)
    wxString subcktName; ///< Subcircuit name extracted from .SUBCKT line
    wxString cachedPath; ///< Local cache file path
    wxString errorMsg;
};


class SPICE_MODEL_DOWNLOADER
{
public:
    SPICE_MODEL_DOWNLOADER( const wxString& aProjectPath );

    /// Check if an MPN matches any known manufacturer source.
    bool HasSource( const wxString& aMpn ) const;

    /// Fetch model: check cache first, then download.
    SPICE_MODEL_RESULT FetchModel( const wxString& aMpn );

    /// Check if a cached model exists for the given MPN.
    bool HasCachedModel( const wxString& aMpn ) const;

    /// Read a cached model from disk.
    SPICE_MODEL_RESULT ReadCachedModel( const wxString& aMpn ) const;

    /// Extract the .SUBCKT name from SPICE text.
    static wxString ExtractSubcktName( const wxString& aSpiceText );

private:
    struct MODEL_SOURCE
    {
        wxString              name;
        std::vector<wxString> mpnPatterns; ///< regex patterns
        wxString              netlistUrl;  ///< URL template with {mpn}
    };

    wxString            getCachePath( const wxString& aMpn ) const;
    wxString            downloadSpiceText( const wxString& aUrl );
    const MODEL_SOURCE* findSource( const wxString& aMpn ) const;

    wxString                  m_projectPath;
    std::vector<MODEL_SOURCE> m_sources;
};

#endif // SPICE_MODEL_DOWNLOADER_H
