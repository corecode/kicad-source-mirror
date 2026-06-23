/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.TXT for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you may find one at
 * http://www.gnu.org/licenses/
 */

#include <qa_utils/wx_utils/unit_test_utils.h>
#include <schematic_utils/schematic_file_util.h>

#include <connection_graph.h>
#include <schematic.h>
#include <erc/erc_settings.h>
#include <erc/erc.h>
#include <erc/erc_report.h>
#include <settings/settings_manager.h>
#include <locale_io.h>


struct ERC_ALIAS_TEST_FIXTURE
{
    ERC_ALIAS_TEST_FIXTURE() {}

    SETTINGS_MANAGER           m_settingsManager;
    std::unique_ptr<SCHEMATIC> m_schematic;
};


static int countErrors( SCHEMATIC* aSchematic, int aSeverities )
{
    SHEETLIST_ERC_ITEMS_PROVIDER errors( aSchematic );
    errors.SetSeverities( aSeverities );
    return errors.GetCount();
}


static void suppressUnrelatedChecks( ERC_SETTINGS& settings )
{
    // Suppress all checks except the ones under test
    for( int i = ERCE_FIRST; i <= ERCE_LAST; ++i )
        settings.m_ERCSeverities[i] = RPT_SEVERITY_IGNORE;
}


BOOST_FIXTURE_TEST_CASE( ERCAliasLabelNoDriverConflict, ERC_ALIAS_TEST_FIXTURE )
{
    LOCALE_IO dummy;

    // A net with label "SDA" and alias "=PB8" should produce no ERCE_DRIVER_CONFLICT
    KI_TEST::LoadSchematic( m_settingsManager, "erc_alias_no_conflict", m_schematic );

    ERC_SETTINGS& settings = m_schematic->ErcSettings();
    suppressUnrelatedChecks( settings );
    settings.m_ERCSeverities[ERCE_DRIVER_CONFLICT] = RPT_SEVERITY_ERROR;

    m_schematic->ConnectionGraph()->RunERC();

    ERC_REPORT reportWriter( m_schematic.get(), EDA_UNITS::MM );
    int        errors = countErrors( m_schematic.get(), RPT_SEVERITY_ERROR );

    BOOST_CHECK_MESSAGE( errors == 0, "Expected 0 driver conflict errors but got " << errors << "\n"
                                                                                   << reportWriter.GetTextReport() );
}


BOOST_FIXTURE_TEST_CASE( ERCAliasLabelSimilarity, ERC_ALIAS_TEST_FIXTURE )
{
    LOCALE_IO dummy;

    // "=PB8" on net A and "PB8" on net B should produce ERCE_ALIAS_LABEL_SIMILARITY
    KI_TEST::LoadSchematic( m_settingsManager, "erc_alias_similarity", m_schematic );

    ERC_SETTINGS& settings = m_schematic->ErcSettings();
    suppressUnrelatedChecks( settings );
    settings.m_ERCSeverities[ERCE_ALIAS_LABEL_SIMILARITY] = RPT_SEVERITY_ERROR;

    m_schematic->ConnectionGraph()->RunERC();

    ERC_REPORT reportWriter( m_schematic.get(), EDA_UNITS::MM );
    int        errors = countErrors( m_schematic.get(), RPT_SEVERITY_ERROR );

    BOOST_CHECK_MESSAGE( errors == 1, "Expected 1 alias similarity error but got " << errors << "\n"
                                                                                   << reportWriter.GetTextReport() );
}


BOOST_FIXTURE_TEST_CASE( ERCAliasWithoutDriver, ERC_ALIAS_TEST_FIXTURE )
{
    LOCALE_IO dummy;

    // "=PB8" alone on a net should produce ERCE_ALIAS_WITHOUT_DRIVER
    KI_TEST::LoadSchematic( m_settingsManager, "erc_alias_without_driver", m_schematic );

    ERC_SETTINGS& settings = m_schematic->ErcSettings();
    suppressUnrelatedChecks( settings );
    settings.m_ERCSeverities[ERCE_ALIAS_WITHOUT_DRIVER] = RPT_SEVERITY_ERROR;

    m_schematic->ConnectionGraph()->RunERC();

    ERC_REPORT reportWriter( m_schematic.get(), EDA_UNITS::MM );
    int        errors = countErrors( m_schematic.get(), RPT_SEVERITY_ERROR );

    BOOST_CHECK_MESSAGE( errors == 1, "Expected 1 alias-without-driver error but got "
                                              << errors << "\n"
                                              << reportWriter.GetTextReport() );
}


BOOST_FIXTURE_TEST_CASE( ERCAliasMultipleOnSameNet, ERC_ALIAS_TEST_FIXTURE )
{
    LOCALE_IO dummy;

    // Multiple aliases with a real driver on the same net — no errors
    KI_TEST::LoadSchematic( m_settingsManager, "erc_alias_multiple", m_schematic );

    ERC_SETTINGS& settings = m_schematic->ErcSettings();
    suppressUnrelatedChecks( settings );
    settings.m_ERCSeverities[ERCE_DRIVER_CONFLICT] = RPT_SEVERITY_ERROR;
    settings.m_ERCSeverities[ERCE_ALIAS_LABEL_SIMILARITY] = RPT_SEVERITY_ERROR;
    settings.m_ERCSeverities[ERCE_ALIAS_WITHOUT_DRIVER] = RPT_SEVERITY_ERROR;

    m_schematic->ConnectionGraph()->RunERC();

    ERC_REPORT reportWriter( m_schematic.get(), EDA_UNITS::MM );
    int        errors = countErrors( m_schematic.get(), RPT_SEVERITY_ERROR );

    BOOST_CHECK_MESSAGE( errors == 0, "Expected 0 errors but got " << errors << "\n" << reportWriter.GetTextReport() );
}


BOOST_FIXTURE_TEST_CASE( ERCAliasHierarchicalPriority, ERC_ALIAS_TEST_FIXTURE )
{
    LOCALE_IO dummy;

    // A subsheet has hier label "NET1" and alias "=PB8" on the same wire.
    // The parent connects via sheet pin "NET1" with local label "PARENT_NET".
    // After hierarchy propagation, the net name should be "PARENT_NET"
    // (from the parent's local label), NOT "=PB8" (from the alias).
    KI_TEST::LoadSchematic( m_settingsManager, "erc_alias_hier_priority", m_schematic );

    CONNECTION_GRAPH* graph = m_schematic->ConnectionGraph();
    SCH_SHEET_LIST    sheets = m_schematic->BuildSheetListSortedByPageNumbers();

    // Find the subsheet path
    SCH_SHEET_PATH subSheetPath;

    for( const SCH_SHEET_PATH& path : sheets )
    {
        if( path.size() > 1 )
        {
            subSheetPath = path;
            break;
        }
    }

    BOOST_REQUIRE_MESSAGE( subSheetPath.size() > 1, "Could not find subsheet path" );

    // The net in the subsheet should have been named "PARENT_NET" by
    // hierarchy propagation, not "=PB8" from the alias label.
    // FindSubgraphByName expects the full path-qualified name from Connection::Name().
    CONNECTION_SUBGRAPH* sg =
            graph->FindSubgraphByName( "/PARENT_NET", subSheetPath );

    BOOST_CHECK_MESSAGE( sg != nullptr,
                         "Expected net 'PARENT_NET' in subsheet but it was not found"
                                 " — alias label may have overridden the hierarchical name" );
}
