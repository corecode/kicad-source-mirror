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

#include "spice_model_downloader.h"

#include <kicad_curl/kicad_curl_easy.h>

#include <wx/filename.h>
#include <wx/regex.h>
#include <wx/textfile.h>
#include <wx/tokenzr.h>


SPICE_MODEL_DOWNLOADER::SPICE_MODEL_DOWNLOADER( const wxString& aProjectPath ) :
        m_projectPath( aProjectPath )
{
    // Register built-in Murata source as default
    MODEL_SOURCE murata;
    murata.name = wxS( "Murata" );
    murata.mpnPatterns = { wxS( "^BLM" ), wxS( "^BLE" ), wxS( "^BLF" ), wxS( "^BLT" ),
                           wxS( "^NFZ" ), wxS( "^NFM" ), wxS( "^GRM" ), wxS( "^GCM" ) };
    murata.netlistUrl = wxS( "https://ds.murata.com/simserve/characteristics?modeltype=undefined"
                             "&partnumbers={mpn},&ReqType=GetNetList" );
    m_sources.push_back( std::move( murata ) );
}


const SPICE_MODEL_DOWNLOADER::MODEL_SOURCE*
SPICE_MODEL_DOWNLOADER::findSource( const wxString& aMpn ) const
{
    wxString mpnUpper = aMpn.Upper();

    for( const MODEL_SOURCE& src : m_sources )
    {
        for( const wxString& pattern : src.mpnPatterns )
        {
            wxRegEx re( pattern, wxRE_ICASE );

            if( re.IsValid() && re.Matches( mpnUpper ) )
                return &src;
        }
    }

    return nullptr;
}


bool SPICE_MODEL_DOWNLOADER::HasSource( const wxString& aMpn ) const
{
    return findSource( aMpn ) != nullptr;
}


wxString SPICE_MODEL_DOWNLOADER::getCachePath( const wxString& aMpn ) const
{
    wxFileName fn;
    fn.SetPath( m_projectPath );
    fn.AppendDir( wxS( "spice_models" ) );
    fn.SetName( aMpn );
    fn.SetExt( wxS( "lib" ) );
    return fn.GetFullPath();
}


bool SPICE_MODEL_DOWNLOADER::HasCachedModel( const wxString& aMpn ) const
{
    return wxFileName::FileExists( getCachePath( aMpn ) );
}


SPICE_MODEL_RESULT SPICE_MODEL_DOWNLOADER::ReadCachedModel( const wxString& aMpn ) const
{
    SPICE_MODEL_RESULT result = { false, wxEmptyString, wxEmptyString, wxEmptyString,
                                  wxEmptyString };

    wxString path = getCachePath( aMpn );

    if( !wxFileName::FileExists( path ) )
    {
        result.errorMsg = wxString::Format( wxS( "Cached model not found: %s" ), path );
        return result;
    }

    wxTextFile file( path );

    if( !file.Open() )
    {
        result.errorMsg = wxString::Format( wxS( "Failed to read: %s" ), path );
        return result;
    }

    wxString text;

    for( wxString line = file.GetFirstLine(); !file.Eof(); line = file.GetNextLine() )
    {
        if( !text.IsEmpty() )
            text += wxS( "\n" );

        text += line;
    }

    if( !text.IsEmpty() )
        text += wxS( "\n" );

    file.Close();

    wxString subcktName = ExtractSubcktName( text );

    if( subcktName.IsEmpty() )
    {
        result.errorMsg = wxString::Format( wxS( "No .SUBCKT found in %s" ), path );
        return result;
    }

    result.success = true;
    result.spiceText = text;
    result.subcktName = subcktName;
    result.cachedPath = path;
    return result;
}


wxString SPICE_MODEL_DOWNLOADER::ExtractSubcktName( const wxString& aSpiceText )
{
    static wxRegEx subcktRe( wxS( "^\\.subckt\\s+(\\S+)" ), wxRE_ICASE | wxRE_NEWLINE );

    if( subcktRe.Matches( aSpiceText ) )
        return subcktRe.GetMatch( aSpiceText, 1 );

    return wxString();
}


wxString SPICE_MODEL_DOWNLOADER::downloadSpiceText( const wxString& aUrl )
{
    KICAD_CURL_EASY curl;

    curl.SetURL( aUrl.ToStdString() );
    curl.SetUserAgent( "KiCad-PDN-Analyzer" );
    curl.SetFollowRedirects( true );
    curl.SetConnectTimeout( 10 );

    int rc = curl.Perform();

    if( rc != 0 )
        return wxString();

    int httpCode = curl.GetResponseStatusCode();

    if( httpCode != 200 )
        return wxString();

    const std::string& buf = curl.GetBuffer();

    if( buf.empty() )
        return wxString();

    return wxString::FromUTF8( buf );
}


SPICE_MODEL_RESULT SPICE_MODEL_DOWNLOADER::FetchModel( const wxString& aMpn )
{
    SPICE_MODEL_RESULT result = { false, wxEmptyString, wxEmptyString, wxEmptyString,
                                  wxEmptyString };

    // Check cache first
    if( HasCachedModel( aMpn ) )
        return ReadCachedModel( aMpn );

    const MODEL_SOURCE* src = findSource( aMpn );

    if( !src )
    {
        result.errorMsg = wxString::Format( wxS( "No known source for MPN '%s'" ), aMpn );
        return result;
    }

    // Build URL from template
    wxString url = src->netlistUrl;
    url.Replace( wxS( "{mpn}" ), aMpn );

    // Download
    wxString spiceText = downloadSpiceText( url );

    // If empty, try stripping the last character (packaging suffix)
    if( spiceText.IsEmpty() || ExtractSubcktName( spiceText ).IsEmpty() )
    {
        if( aMpn.length() > 4 )
        {
            wxString trimmedMpn = aMpn.Left( aMpn.length() - 1 );
            wxString trimmedUrl = src->netlistUrl;
            trimmedUrl.Replace( wxS( "{mpn}" ), trimmedMpn );

            spiceText = downloadSpiceText( trimmedUrl );
        }
    }

    if( spiceText.IsEmpty() )
    {
        result.errorMsg = wxString::Format( wxS( "Empty response for MPN '%s'" ), aMpn );
        return result;
    }

    wxString subcktName = ExtractSubcktName( spiceText );

    if( subcktName.IsEmpty() )
    {
        result.errorMsg = wxString::Format( wxS( "No .SUBCKT in response for MPN '%s'" ), aMpn );
        return result;
    }

    // Save to cache
    wxString   cachePath = getCachePath( aMpn );
    wxFileName cacheDir;
    cacheDir.SetPath( m_projectPath );
    cacheDir.AppendDir( wxS( "spice_models" ) );

    if( !cacheDir.DirExists() )
        cacheDir.Mkdir( wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL );

    wxTextFile file( cachePath );

    if( file.Create() || file.Open() )
    {
        file.Clear();

        // Split on newlines and add each line
        wxStringTokenizer tokenizer( spiceText, wxS( "\n" ), wxTOKEN_RET_EMPTY );

        while( tokenizer.HasMoreTokens() )
            file.AddLine( tokenizer.GetNextToken() );

        file.Write();
        file.Close();
    }

    result.success = true;
    result.spiceText = spiceText;
    result.subcktName = subcktName;
    result.cachedPath = cachePath;
    return result;
}
