//#define FBX_COMPATIBILITY_MODE DzFbxImporter::ECompatibilityMode::DefaultCompatibilityMode
#define FBX_COMPATIBILITY_MODE	DzFbxImporter::ECompatibilityMode::UniversalCompatbilityMode

/**********************************************************************
	Copyright (C) 2012-2022 DAZ 3D, Inc. All Rights Reserved.

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
**********************************************************************/
#include "dzversion.h"

#if ((DZ_SDK_VERSION_MAJOR >= 5) || ((DZ_SDK_VERSION_MAJOR == 4) && (DZ_SDK_VERSION_MINOR >= 12)))
#define DZ_SDK_4_12_OR_GREATER 1
#else
#define DZ_SDK_4_12_OR_GREATER 0
#endif

/*****************************
	Include files
*****************************/
// Direct Relation
#include "DzFbxImporter.h"

// System

// Standard Library

// Qt
#include <QtGui/QComboBox>

// DS Public SDK
#include "dzapp.h"
#include "dzbone.h"
#include "dzbonebinding.h"
#if DZ_SDK_4_12_OR_GREATER
#include "dzcollapsiblegroupbox.h"
#endif //DZ_SDK_4_12_OR_GREATER
#include "dzdefaultmaterial.h"
#include "dzenumproperty.h"
#include "dzfacegroup.h"
#include "dzfacetmesh.h"
#include "dzfacetshape.h"
#include "dzfigure.h"
#include "dzfileiosettings.h"
#include "dzfloatproperty.h"
#if DZ_SDK_4_12_OR_GREATER
#include "dzgraftingfigureshape.h"
#endif // DZ_SDK_4_12_OR_GREATER
#include "dzdynamicdividerwgt.h"
#include "dzimagemgr.h"
#include "dzmorph.h"
#include "dzmorphdeltas.h"
#include "dznode.h"
#include "dzobject.h"
#include "dzpresentation.h"
#include "dzprogress.h"
#include "dzscene.h"
#include "dzselectionmap.h"
#include "dzsettings.h"
#include "dzskinbinding.h"
#include "dzstyle.h"

// Project Specific

// DB 2023-Mar-30: In-house Fbx utility library
#include "FbxTools.h"

/*****************************
	Local Definitions
*****************************/

#if FBXSDK_VERSION_MAJOR >= 2016
#define DATA_FBX_USER_PROPERTIES "FbxUserProperties"
#define DATA_LOD_INFO "LODInfo"
#endif

namespace
{

// settings keys
const QString c_optTake( "Take" );

const QString c_optIncRotationLimits( "IncludeRotationLimits" );
const QString c_optIncAnimations( "IncludeAnimations" );

const QString c_optIncPolygonSets( "IncludePolygonSets" );
const QString c_optIncPolygonGroups( "IncludePolygonGroups" );

const QString c_optStudioNodeNamesLabels( "IncludeNodeNamesLabels" );
const QString c_optStudioPresentation( "IncludeNodePresentation" );
const QString c_optStudioNodeSelectionMap( "IncludeNodeSelectionMap" );
const QString c_optStudioSceneIDs( "IncludeSceneIDs" );

const QString c_optRunSilent( "RunSilent" );

// settings default values
const bool c_defaultIncludeRotationLimits = true;
const bool c_defaultIncludeAnimations = false;

const bool c_defaultIncludePolygonSets = true;
const bool c_defaultIncludePolygonGroups = false;

const bool c_defaultStudioNodeNames = true;
const bool c_defaultStudioNodePresentation = true;
const bool c_defaultStudioNodeSelectionMap = true;
const bool c_defaultStudioSceneIDs = true;

// functions
DzFigure* createFigure()
{
	DzFigure* dsFigure = new DzFigure();

	DzEnumProperty* followModeControl = NULL;
#if DZ_SDK_4_12_OR_GREATER
	followModeControl = dsFigure->getFollowModeControl();
	followModeControl->setValue( DzSkeleton::fmAutoFollow );
#else
	// DzSkeleton::getFollowModeControl() is not in the 4.5 SDK, so we attempt
	// to use the meta-object to call the methods - since 4.8.0.23. If this fails,
	// we attempt to find the property by name and if found set its value.

	if ( !QMetaObject::invokeMethod( dsFigure,
		"getFollowModeControl", Q_RETURN_ARG( DzEnumProperty*, followModeControl ) )
		|| !followModeControl )
	{
		followModeControl = qobject_cast<DzEnumProperty*>( dsFigure->findProperty( "Fit to Mode" ) );
	}
	if (followModeControl)
	{
		followModeControl->setValue(1);
	}

#endif

	dsFigure->setDrawGLBones( true );

	return dsFigure;
}

} // namespace

///////////////////////////////////////////////////////////////////////
// DzFbxImporter
///////////////////////////////////////////////////////////////////////

#ifdef DAZ_SCRIPT_DOCS

class DzFbxImporter : public DzImporter {
public:
// DAZScript Constructors

	DzFbxImporter();
};

#endif // DAZ_SCRIPT_DOCS

/**
	@script
	@class	DzFbxImporter

	@ingroup	FileIO
	@nosubgrouping
	@brief		Class for importing files in the Autodesk FBX (*.fbx) format.

	A specialization of DzImporter that implements an importer for the Autodesk
	FBX (.fbx) format. Through its use of the FBX SDK to accomplish this task,
	this importer also provides import capabilities for the Autodesk AutoCAD DXF
	(.dxf) format, the Autodesk 3ds Max (.3ds) format, and the Collada DAE (.dae)
	format, as supported by the FBX SDK.

	@attention	The FBX SDK also provides import capabilities for the Alias
				Wavefront OBJ (.obj) format, but it has been intentionally
				excluded. Use DzObjImporter instead.

	@sa DzImportMgr::findImporterByClassName()
	@sa DzImportMgr::findImporter()

	@sa Sample: @dzscript{api_reference/samples/file_io/import_fbx_silent/start, Silent FBX Import}
**/


/**
	@script
	Default Constructor.
**/
DzFbxImporter::DzFbxImporter() :
	m_fbxRead( false ),
	m_fbxManager( NULL ),
	m_fbxScene( NULL ),
	m_fbxAnimStack( NULL ),
	m_fbxAnimLayer( NULL ),
	m_fbxFileMajor( 0 ),
	m_fbxFileMinor( 0 ),
	m_fbxFileRevision( 0 ),
	m_fbxFileBinary( -1 ),
	m_needConversion( false ),
	m_dsEndTime( 0 ),
	m_suppressRigErrors( false ),
	m_includeRotationLimits( c_defaultIncludeRotationLimits ),
	m_includeAnimations( c_defaultIncludeAnimations ),
	m_includePolygonSets( c_defaultIncludePolygonSets ),
	m_includePolygonGroups( c_defaultIncludePolygonGroups ),
	m_studioNodeNamesLabels( c_defaultStudioNodeNames ),
	m_studioNodePresentation( c_defaultStudioNodePresentation ),
	m_studioNodeSelectionMap( c_defaultStudioNodeSelectionMap ),
	m_studioSceneIDs( c_defaultStudioSceneIDs ),
	m_root( NULL ),
	//// DB 4-14-2023: compatibility with fbx from other software
	m_compatibilityMode( FBX_COMPATIBILITY_MODE )
{
	//// DB 4-15-2023: support different axis systems
	m_axisCorrectionMatrix.SetIdentity();
}

/**
**/
DzFbxImporter::~DzFbxImporter()
{
}

/**
**/
bool DzFbxImporter::recognize( const QString &filename ) const
{
	const QString ext = getFileExtension( filename ); //return value is lowercase
	for ( int i = 0, n = getNumExtensions(); i < n; i++ )
	{
		if ( ext == getExtension( i ) )
		{
			return true;
		}
	}

	return false;
}

/**
**/
int DzFbxImporter::getNumExtensions() const
{
	return 4;
}

/**
**/
QString	DzFbxImporter::getExtension( int i ) const
{
	switch ( i )
	{
	case 0:
		return "fbx";
	case 1:
		return "dxf";
	case 2:
		return "3ds";
	case 3:
		// DzCOLLADAImporter depends on FCollada (discontinued)
		// DzCOLLADAImporter is deprecated
		return "dae";
	case 4:
		// DzObjImporter provides a more suitable result
		// This conflicts with DzObjImporter and recognize()
		//return "obj";
	default:
		return QString();
	}
}

/**
**/
QString	DzFbxImporter::getDescription() const
{
	return "Autodesk FBX SDK";
}

/**
**/
void DzFbxImporter::getDefaultOptions( DzFileIOSettings* options ) const
{
	assert( options );
	if ( !options )
	{
		return;
	}

	// Properties
	options->setBoolValue( c_optIncRotationLimits, c_defaultIncludeRotationLimits );
	options->setBoolValue( c_optIncAnimations, c_defaultIncludeAnimations );
	options->setStringValue( c_optTake, QString() );

	// Geometry
	options->setBoolValue( c_optIncPolygonSets, c_defaultIncludePolygonSets );
	options->setBoolValue( c_optIncPolygonGroups, c_defaultIncludePolygonGroups );

	// Custom Data
	options->setBoolValue( c_optStudioNodeNamesLabels, c_defaultStudioNodeNames );
	options->setBoolValue( c_optStudioPresentation, c_defaultStudioNodePresentation );
	options->setBoolValue( c_optStudioNodeSelectionMap, c_defaultStudioNodeSelectionMap );
	options->setBoolValue( c_optStudioSceneIDs, c_defaultStudioSceneIDs );

	options->setIntValue( c_optRunSilent, 0 );
}


namespace
{

class MapConversion {
public:
	QVector<double> fbxWeights;
	unsigned short* dsWeights;
};

bool isChildNode( DzNode* child, DzNode* parent )
{
	if ( !parent || !child || child == parent )
	{
		return false;
	}

	while ( child )
	{
		if ( child == parent )
		{
			return true;
		}
		child = child->getNodeParent();
	}

	return false;
}

} //namespace

/**
	Manually get the options. If the "RunSilent" option is true, then the dialog
	will be skipped.
**/
int DzFbxImporter::getOptions( DzFileIOSettings* options, const DzFileIOSettings* impOptions, const QString &filename )
{
	assert( options );
	assert( impOptions );
	if ( !options || !impOptions )
	{
		return false;
	}

	bool optionsShown = false;
#if DZ_SDK_4_12_OR_GREATER
	optionsShown = getOptionsShown();
#endif

	if ( optionsShown || impOptions->getIntValue( c_optRunSilent, 0 ) )
	{
		if ( optionsShown )
		{
			getSavedOptions( options ); // includes defaults
		}
		else
		{
			getDefaultOptions( options );
		}

#if DZ_SDK_4_12_OR_GREATER
		copySettings( options, impOptions );
#else
		// DzFileIO::copySettings() is not in the 4.5 SDK, and it is not exposed
		// to QMetaObject::invokedMethod() in later builds, so we copy manually.
		for ( int i = 0, n = options->getNumValues(); i < n; i++ )
		{
			const QString key = options->getKey( i );
			switch ( options->getValueType( i ) )
			{
			default:
			case DzSettings::StringValue:
			case DzSettings::IntValue:
			case DzSettings::BoolValue:
			case DzSettings::FloatValue:
				impOptions->copySetting( key, options );
				break;
			case DzSettings::SettingsValue:
				impOptions->copySetting( key, options->getSettingsValue( key ) );
				break;
			}
		}
#endif

		return true;
	}

	fbxRead( filename );
	fbxPreImport();

	DzFbxImporter* self = const_cast<DzFbxImporter*>( this );
	DzFbxImportFrame* frame = new DzFbxImportFrame( self );
	if ( !frame )
	{
		fbxCleanup();
		return true;
	}

	DzFileIODlg optionsDlg( frame );
	frame->setOptions( impOptions, filename );
	if ( optionsDlg.exec() != QDialog::Accepted )
	{
		fbxCleanup();
		return false; // user cancelled
	}

#if DZ_SDK_4_12_OR_GREATER
	setOptionsShown( true );
#endif

	frame->getOptions( options );

	// if handling the options dialog ourselves, we also need to save the state
	options->setIntValue( c_optRunSilent, 0 );
	saveOptions( options );

	return true;
}

/**
**/
void DzFbxImporter::fbxRead( const QString &filename )
{
	if ( m_fbxRead )
	{
		return;
	}

	QString orgName;
#if DZ_SDK_4_12_OR_GREATER
	orgName = dzApp->getOrgName();
#else
	// DzApp::getOrgName() is not in the 4.5 SDK, so we attempt to use the
	// meta-object to access the DzApp::orgName property - since 4.6.4.30.

	orgName = dzApp->property( "orgName" ).toString();
#endif
	if ( !orgName.isEmpty()
		&& orgName != QString( "DAZ 3D" ) )
	{
		m_suppressRigErrors = true;
	}

	m_fbxManager = FbxManager::Create();
	FbxIOSettings* fbxIoSettings = FbxIOSettings::Create( m_fbxManager, IOSROOT );
	m_fbxManager->SetIOSettings( fbxIoSettings );

	m_fbxScene = FbxScene::Create( m_fbxManager, "" );

	m_fbxAnimStack = NULL;
	m_fbxAnimLayer = NULL;
	m_dsEndTime = dzScene->getAnimRange().getEnd();

	FbxImporter* fbxImporter = FbxImporter::Create( m_fbxManager, "" );
	if ( !fbxImporter->Initialize( filename.toUtf8().data(), -1, fbxIoSettings ) )
	{
		const fbxsdk::FbxStatus status = fbxImporter->GetStatus();
		if ( status != FbxStatus::eSuccess )
		{
			dzApp->warning( QString( "FBX Importer: %1" ).arg( status.GetErrorString() ) );
		}
	}

	fbxImporter->GetFileVersion( m_fbxFileMajor, m_fbxFileMinor, m_fbxFileRevision );

	if ( fbxImporter->IsFBX() )
	{
		fbxIoSettings->SetBoolProp( IMP_FBX_MATERIAL, true );
		fbxIoSettings->SetBoolProp( IMP_FBX_TEXTURE, true );
		fbxIoSettings->SetBoolProp( IMP_FBX_LINK, true );
		fbxIoSettings->SetBoolProp( IMP_FBX_SHAPE, true );
		fbxIoSettings->SetBoolProp( IMP_FBX_GOBO, true );
		fbxIoSettings->SetBoolProp( IMP_FBX_ANIMATION, true );
		fbxIoSettings->SetBoolProp( IMP_FBX_GLOBAL_SETTINGS, true );
	}

#if FBXSDK_VERSION_MAJOR >= 2020 && FBXSDK_VERSION_MINOR > 0
	fbxImporter->GetStatus().KeepErrorStringHistory( true );
#endif

	fbxImporter->Import( m_fbxScene );

	const fbxsdk::FbxStatus& status = fbxImporter->GetStatus();
	if ( status != FbxStatus::eSuccess )
	{
#if FBXSDK_VERSION_MAJOR >= 2020 && FBXSDK_VERSION_MINOR > 2
		FbxArray<FbxString*> history;
		status.GetErrorStringHistory( history );
		if ( history.GetCount() > 1 )
		{
			// error strings are in stack order (last error -> first element)
			for ( int i = history.GetCount() - 1; i >= 0 ; --i )
			{
				dzApp->warning( QString( "FBX Importer: %1" ).arg( history[i]->Buffer() ) );
			}
		}
		FbxArrayDelete<FbxString*>( history );
#else
		dzApp->warning( QString( "FBX Importer: %1" ).arg( status.GetErrorString() ) );
#endif
	}

	const FbxIOFileHeaderInfo* fbxHeaderInfo = fbxImporter->GetFileHeaderInfo();
	m_fbxFileCreator = fbxHeaderInfo->mCreator;
#if FBXSDK_VERSION_MAJOR > 2020 || (FBXSDK_VERSION_MAJOR == 2020 && FBXSDK_VERSION_MINOR >= 3)
	m_fbxFileBinary = fbxHeaderInfo->mBinary ? 1 : 0;
#endif

	fbxImporter->Destroy();

	const FbxDocumentInfo* fbxSceneInfo = m_fbxScene->GetSceneInfo();
	m_fbxSceneAuthor = fbxSceneInfo->mAuthor;
	m_fbxSceneTitle = fbxSceneInfo->mTitle;
	m_fbxSceneSubject = fbxSceneInfo->mSubject;
	m_fbxSceneKeywords = fbxSceneInfo->mKeywords;
	m_fbxSceneRevision = fbxSceneInfo->mRevision;
	m_fbxSceneComment = fbxSceneInfo->mComment;
	m_fbxOrigAppVendor = fbxSceneInfo->Original_ApplicationVendor;
	m_fbxOrigAppName = fbxSceneInfo->Original_ApplicationName;
	m_fbxOrigAppVersion = fbxSceneInfo->Original_ApplicationVersion;

	//// DB 4-14-2023: Fbx Global Settings support
	if (m_compatibilityMode)
	{
		FbxGlobalSettings& fbxGlobalSettings = m_fbxScene->GetGlobalSettings();
		fbxsdk::FbxAxisSystem dsAxisSystem(fbxsdk::FbxAxisSystem::eYAxis,
			fbxsdk::FbxAxisSystem::eParityOdd, fbxsdk::FbxAxisSystem::eRightHanded);
		fbxsdk::FbxAxisSystem fbxAxisSystem = fbxGlobalSettings.GetAxisSystem();
		if (fbxAxisSystem != dsAxisSystem)
		{
			// Use ConvertScene() to change nodes, but must also apply change to all vertices because we do not render using Fbx scenegraph
			dsAxisSystem.ConvertScene(m_fbxScene);

			if (m_fbxOrigAppName != "Unreal Engine")
			{
				int fbxUpVectorSign;
				int fbxFrontVectorSign;
				fbxsdk::FbxAxisSystem::EUpVector fbxUpVector = fbxAxisSystem.GetUpVector(fbxUpVectorSign);
				fbxsdk::FbxAxisSystem::EFrontVector fbxFrontVectorParity = fbxAxisSystem.GetFrontVector(fbxFrontVectorSign);
				m_axisCorrectionMatrix.SetIdentity();
				if (fbxUpVector == fbxsdk::FbxAxisSystem::eZAxis)
				{
					if (fbxFrontVectorParity == fbxsdk::FbxAxisSystem::eParityOdd)
					{
						// Y-front
						m_axisCorrectionMatrix[1][1] = 0.0;
						m_axisCorrectionMatrix[1][2] = 1.0 * fbxFrontVectorSign;
						m_axisCorrectionMatrix[2][2] = 0.0;
						m_axisCorrectionMatrix[2][1] = 1.0 * fbxUpVectorSign;
					}
					else
					{
						// X-front
						m_axisCorrectionMatrix[0][0] = 0.0;
						m_axisCorrectionMatrix[0][2] = 1.0 * fbxFrontVectorSign;
						m_axisCorrectionMatrix[2][2] = 0.0;
						m_axisCorrectionMatrix[2][1] = 1.0 * fbxUpVectorSign;
					}
				}
				else if (fbxUpVector == fbxsdk::FbxAxisSystem::eYAxis)
				{
					if (fbxFrontVectorParity == fbxsdk::FbxAxisSystem::eParityOdd)
					{
						// Z-front, so should only need to change Up and Front vector signs
						m_axisCorrectionMatrix[1][1] = 1.0 * fbxUpVectorSign;
						m_axisCorrectionMatrix[2][2] = 1.0 * fbxFrontVectorSign;
					}
					else
					{
						// X-front
						// set up axis sign
						m_axisCorrectionMatrix[1][1] = 1.0 * fbxUpVectorSign;
						// transpose X and Z axis
						m_axisCorrectionMatrix[0][0] = 0.0;
						m_axisCorrectionMatrix[0][2] = 1.0 * fbxFrontVectorSign;
						m_axisCorrectionMatrix[2][2] = 0.0;
						m_axisCorrectionMatrix[2][0] = 1.0;
					}
				}
				else
				{
					// this is theoretical axis system, should not be used in practice

					if (fbxFrontVectorParity == fbxsdk::FbxAxisSystem::eParityOdd)
					{
						// X-up, Z-front
						// transpose x and y
						m_axisCorrectionMatrix[0][0] = 0.0;
						m_axisCorrectionMatrix[0][1] = 1.0 * fbxUpVectorSign;
						m_axisCorrectionMatrix[1][1] = 0.0;
						m_axisCorrectionMatrix[1][0] = 1.0;
						// set z sign
						m_axisCorrectionMatrix[2][2] = 1.0 * fbxFrontVectorSign;
					}
					else
					{
						// X-up, Y-front
						// transpose x to y...
						m_axisCorrectionMatrix[0][0] = 0.0;
						m_axisCorrectionMatrix[0][1] = 1.0 * fbxUpVectorSign;
						// transpose y to z...
						m_axisCorrectionMatrix[1][1] = 0.0;
						m_axisCorrectionMatrix[1][2] = 1.0 * fbxFrontVectorSign;
						// transpose z to x
						m_axisCorrectionMatrix[2][2] = 0.0;
						m_axisCorrectionMatrix[2][0] = 1.0;
					}
				}
			}

		}
		else
		{
			// Assume that no AxisSystem was provided...
			// Override the Default Axis Settings for 3ds Max
			//if (m_fbxOrigAppName == "3ds Max" || m_fbxOrigAppName == "Maya")
			if (m_fbxOrigAppName == "3ds Max")
			{
				int fbxUpVectorSign = 1;
				int fbxFrontVectorSign = -1;
				// Assume +Y up, -Z forward
				m_axisCorrectionMatrix[1][1] = 0.0;
				m_axisCorrectionMatrix[1][2] = 1.0 * fbxFrontVectorSign;
				m_axisCorrectionMatrix[2][2] = 0.0;
				m_axisCorrectionMatrix[2][1] = 1.0 * fbxUpVectorSign;
			}
		}

		//// Sanity Check, multiply axisCorrectionMatrix by Y-Up and Z-Up Vectors
		//FbxVector4 Y_UpVector(0, 1, 0);
		//FbxVector4 Z_ForwardVector(0, 0, 1);
		//FbxVector4 newYUpVector = m_axisCorrectionMatrix.MultT(Y_UpVector);
		//FbxVector4 newZForwardVector = m_axisCorrectionMatrix.MultT(Z_ForwardVector);
	}

	m_fbxRead = true;
}

/**
**/
void DzFbxImporter::fbxPickAnimationTake( int idx )
{
	m_fbxAnimStack = m_fbxScene->GetSrcObject<FbxAnimStack>( idx );
	if ( m_fbxAnimStack->GetMemberCount<FbxAnimLayer>() <= 0 )
	{
		return;
	}

	m_fbxAnimLayer = m_fbxAnimStack->GetMember<FbxAnimLayer>( 0 );
}

/**
**/
void DzFbxImporter::fbxPickAnimation()
{
	if ( !m_includeAnimations
		|| m_takeName.isEmpty() )
	{
		return;
	}

	const QString idxPrefix( "idx::" );
	if ( m_takeName.startsWith( idxPrefix ) )
	{
		bool isNum = false;
		const int takeIdx = m_takeName.mid( idxPrefix.length() ).toInt( &isNum );
		if ( isNum
			&& takeIdx > -1
			&& takeIdx < m_fbxScene->GetSrcObjectCount<FbxAnimStack>() )
		{
			fbxPickAnimationTake( takeIdx );
		}
	}
	else
	{
		for ( int i = 0, n = m_fbxScene->GetSrcObjectCount<FbxAnimStack>(); i < n; i++ )
		{
			const FbxAnimStack* animStack = m_fbxScene->GetSrcObject<FbxAnimStack>( i );
			if ( QString( animStack->GetName() ) != m_takeName )
			{
				continue;
			}

			fbxPickAnimationTake( i );
		}
	}
}

/**
**/
void DzFbxImporter::fbxImportSkinning()
{
	QVector<DzBone*> dsBones;
	for ( int i = 0; i < m_skins.size(); i++ )
	{
		Skinning skinning = m_skins[i];

		const Node* node = skinning.node;
		Q_UNUSED( node )

		FbxSkin* fbxSkin = skinning.fbxSkin;
		DzFigure* dsFigure = skinning.dsFigure;
		const int numVertices = skinning.numVertices;

		DzSkinBinding* dsSkin = dsFigure->getSkinBinding();

#if DZ_SDK_4_12_OR_GREATER
			if ( dsSkin
				&& dsSkin->getTargetVertexCount() < 1 )
			{
				dsSkin->setTargetVertexCount( numVertices );
#else
		// DzSkinBinding::getTargetVertexCount() is not in the 4.5 SDK, so
		// we attempt to use the meta-object to call the method.

		int targetVertexCount = 0;
		if ( dsSkin
			&& QMetaObject::invokeMethod( dsSkin, "getTargetVertexCount",
				Q_RETURN_ARG( int, targetVertexCount ) )
			&& targetVertexCount < 1 )
		{
			// DzSkinBinding::setTargetVertexCount() is not in the 4.5 SDK,
			// so we attempt to use the meta-object to call the method.

			bool im = QMetaObject::invokeMethod( dsSkin, "setTargetVertexCount",
				Q_ARG( int, numVertices ) );
			assert( im );
#endif
		}
		else if ( !dsSkin )
		{
			assert( !"Binding was not found" );
			continue;
		}

		const int numClusters = fbxSkin->GetClusterCount();

		DzSkeleton* dsBaseSkeleton = NULL;

		for ( int j = 0; j < numClusters; j++ )
		{
			FbxCluster* fbxCluster = fbxSkin->GetCluster( j );
			DzBone* dsBone = qobject_cast<DzBone*>( m_nodeMap.value( fbxCluster->GetLink() ) );
			if ( !dsBone )
			{
				continue;
			}

			if ( !isChildNode( dsBone, dsFigure ) )
			{
				dsBaseSkeleton = dsBone->getSkeleton();
			}
		}

		if ( dsBaseSkeleton )
		{
			replicateSkeleton( dsBaseSkeleton, skinning );
		}


		DzWeightMapList maps;
		QVector<MapConversion> mapConversions;
		for ( int j = 0; j < numClusters; j++ )
		{
			FbxCluster* fbxCluster = fbxSkin->GetCluster( j );
			DzBone* dsBone = qobject_cast<DzBone*>( m_nodeMap.value( fbxCluster->GetLink() ) );
			if ( !dsBone )
			{
				continue;
			}

			DzBoneBinding* dsBinding = new DzBoneBinding();
			dsBinding->setBone( dsBone );
			dsSkin->addBoneBinding( dsBinding );

			DzWeightMap* dsWeightMap  = new DzWeightMap( numVertices );
			int* fbxIndices = fbxCluster->GetControlPointIndices();
			double* fbxWeights = fbxCluster->GetControlPointWeights();

			MapConversion mapConv;
			mapConv.dsWeights = dsWeightMap->getWeights();
			mapConv.fbxWeights.resize( numVertices );
			for ( int k = 0; k < fbxCluster->GetControlPointIndicesCount(); k++ )
			{
				mapConv.fbxWeights[fbxIndices[k]] = fbxWeights[k];
			}
			mapConversions.append( mapConv );

			dsBinding->setWeights( dsWeightMap );
			FbxAMatrix fbxMatrix;
			fbxCluster->GetTransformLinkMatrix( fbxMatrix );

			// DB 2023-Mar-31: compatibility for Fbx models with joint orientation in BindMatrix
			if (m_compatibilityMode)
			{
				FbxVector4 rotation = fbxMatrix.GetR();
				if (rotation != FbxVector4(0, 0, 0, 0))
				{
					FbxVector4 translation = fbxMatrix.GetT();
					fbxMatrix.SetIdentity();
					fbxMatrix.SetT(translation);
				}
			}

			// DB 2023-Mar-31: compatibility for Fbx models with joint orientation in BindMatrix
			if (m_compatibilityMode)
			{
				FbxVector4 rotation = fbxMatrix.GetR();
				if (rotation != FbxVector4(0, 0, 0, 0))
				{
					FbxVector4 translation = fbxMatrix.GetT();
					fbxMatrix.SetIdentity();
					fbxMatrix.SetT(translation);
				}
			}

			// DB 2023-Mar-31: compatibility for Fbx models with joint orientation in BindMatrix
			if (m_compatibilityMode)
			{
				FbxVector4 rotation = fbxMatrix.GetR();
				if (rotation != FbxVector4(0, 0, 0, 0))
				{
					FbxVector4 translation = fbxMatrix.GetT();
					fbxMatrix.SetIdentity();
					fbxMatrix.SetT(translation);
				}
			}

			DzMatrix3 dsMatrix;
			dsMatrix[0][0] =  fbxMatrix[0][0];
			dsMatrix[0][1] =  fbxMatrix[0][1];
			dsMatrix[0][2] =  fbxMatrix[0][2];
			dsMatrix[1][0] =  fbxMatrix[1][0];
			dsMatrix[1][1] =  fbxMatrix[1][1];
			dsMatrix[1][2] =  fbxMatrix[1][2];
			dsMatrix[2][0] =  fbxMatrix[2][0];
			dsMatrix[2][1] =  fbxMatrix[2][1];
			dsMatrix[2][2] =  fbxMatrix[2][2];
			dsMatrix[3][0] = -fbxMatrix[3][0];
			dsMatrix[3][1] = -fbxMatrix[3][1];
			dsMatrix[3][2] = -fbxMatrix[3][2];

			DzVec3 skelOrigin = dsFigure->getOrigin();
			DzVec3 origin = dsBone->getOrigin();
			dsMatrix[3][0] += ( origin[0] - skelOrigin[0] );
			dsMatrix[3][1] += ( origin[1] - skelOrigin[1] );
			dsMatrix[3][2] += ( origin[2] - skelOrigin[2] );
			dsBinding->setBindingMatrix( dsMatrix );
			maps.append( dsWeightMap );
		}

		for ( int v = 0; v < numVertices; v++ )
		{
			double sum = 0.0;
			for ( int m = 0; m < mapConversions.count(); m++ )
			{
				sum += mapConversions[m].fbxWeights[v];
			}

			for ( int m = 0; m < mapConversions.count(); m++ )
			{
				mapConversions[m].dsWeights[v] = static_cast<unsigned short>( mapConversions[m].fbxWeights[v] / sum * DZ_USHORT_MAX );
			}
		}

		DzWeightMap::normalizeMaps( maps );

		FbxSkin::EType fbxSkinningType = fbxSkin->GetSkinningType();

#if DZ_SDK_4_12_OR_GREATER
		DzSkinBinding::BindingMode bindingMode = DzSkinBinding::General;
		DzSkinBinding::GeneralMapMode generalMapMode = DzSkinBinding::Linear;
		switch ( fbxSkinningType )
		{
		default:
			break;
		case FbxSkin::eDualQuaternion:
			generalMapMode = DzSkinBinding::DualQuat;
			break;
		case FbxSkin::eBlend:
			bindingMode = DzSkinBinding::Blended;
			generalMapMode = DzSkinBinding::DualQuat;
			break;
		}

		dsSkin->setBindingMode( bindingMode );
		dsSkin->setGeneralMapMode( generalMapMode );
		dsSkin->setScaleMode( DzSkinBinding::BindingMaps );

#else
		// DzSkinBinding::BindingMode, and DzSkinBinding::GeneralMapMode are not
		// in the 4.5 SDK, so we use the int equivalents.

		int bindingMode = 0;
		int generalMapMode = 0;
		switch ( fbxSkinningType )
		{
		default:
			break;
		case FbxSkin::eDualQuaternion:
			generalMapMode = 1;
			break;
		case FbxSkin::eBlend:
			bindingMode = 2;
			generalMapMode = 1;
			break;
		}

		// DzSkinBinding::setBindingMode(), DzSkinBinding::setScaleMode(),
		// and DzSkinBinding::setGeneralMapMode() are not in the 4.5 SDK,
		// so we attempt to use the meta-object to call these methods.

		bool im = QMetaObject::invokeMethod( dsSkin, "setBindingMode",
			Q_ARG( int, bindingMode ) );
		assert( im );

		im = QMetaObject::invokeMethod( dsSkin, "setGeneralMapMode",
			Q_ARG( int, generalMapMode ) );
		assert( im );

		im = QMetaObject::invokeMethod( dsSkin, "setScaleMode",
			Q_ARG( int, 1 ) );
		assert( im );
#endif

		if ( fbxSkinningType == FbxSkin::eBlend
			&& skinning.blendWeights )
		{
#if DZ_SDK_4_12_OR_GREATER
			dsSkin->setBlendMap( skinning.blendWeights );
			dsSkin->setBlendMode( DzSkinBinding::BlendLinearDualQuat );
#else
			// DzSkinBinding::setBlendMap() and DzSkinBinding::setBlendMode()
			// are not in the 4.5 SDK, so we attempt to use the meta-object to
			// call these methods.

			im = QMetaObject::invokeMethod( dsSkin, "setBlendMap",
				Q_ARG( DzWeightMap*, skinning.blendWeights.operator->() ) );
			assert( im );

			im = QMetaObject::invokeMethod( dsSkin, "setBlendMode",
				Q_ARG( int, 1 ) );
			assert( im );
#endif
		}
	}
}

/**
**/
void DzFbxImporter::fbxImport()
{
	fbxPickAnimation();

	m_root = new Node();
	m_root->fbxNode = m_fbxScene->GetRootNode();

	fbxImportGraph( m_root );

	fbxImportSkinning();

	fbxImportAnimation( m_root );

	if ( m_includeAnimations )
	{
		dzScene->setAnimRange( DzTimeRange( dzScene->getAnimRange().getStart(), m_dsEndTime ) );
		dzScene->setPlayRange( DzTimeRange( dzScene->getAnimRange().getStart(), m_dsEndTime ) );
	}

	QMap<Node*, QString>::iterator nodeFaceGroupIt;
	for ( nodeFaceGroupIt = m_nodeFaceGroupMap.begin(); nodeFaceGroupIt != m_nodeFaceGroupMap.end(); ++nodeFaceGroupIt )
	{
		updateSelectionMap( nodeFaceGroupIt.key() );
	}
}

/**
**/
void DzFbxImporter::fbxCleanup()
{
	if ( m_fbxManager )
	{
		m_fbxManager->Destroy();
	}

	m_fbxManager = NULL;
}

/**
	@param filename		The full path of the file to import.
	@param impOptions	The options to use while importing the file.

	@return	DZ_NO_ERROR if the file was successfully imported.
**/
DzError DzFbxImporter::read( const QString &filename, const DzFileIOSettings* impOptions )
{
	DzFileIOSettings options;
	const int isOK = getOptions( &options, impOptions, filename );
	if ( !isOK )
	{
		return DZ_USER_CANCELLED_OPERATION;
	}

	// Properties
	m_includeRotationLimits = options.getBoolValue( c_optIncRotationLimits, c_defaultIncludeRotationLimits );
	m_includeAnimations = options.getBoolValue( c_optIncAnimations, c_defaultIncludeAnimations );
	m_takeName = options.getStringValue( c_optTake, QString() );

	// Geometry
	m_includePolygonSets = options.getBoolValue( c_optIncPolygonSets, c_defaultIncludePolygonSets );
	m_includePolygonGroups = options.getBoolValue( c_optIncPolygonGroups, c_defaultIncludePolygonGroups );

	// Custom Data
	m_studioNodeNamesLabels = options.getBoolValue( c_optStudioNodeNamesLabels, c_defaultStudioNodeNames );
	m_studioNodePresentation = options.getBoolValue( c_optStudioPresentation, c_defaultStudioNodePresentation );
	m_studioNodeSelectionMap = options.getBoolValue( c_optStudioNodeSelectionMap, c_defaultStudioNodeSelectionMap );
	m_studioSceneIDs = options.getBoolValue( c_optStudioSceneIDs, c_defaultStudioSceneIDs );

#if DZ_SDK_4_12_OR_GREATER
	clearImportedNodes();
#endif

	m_folder = filename;
	m_folder.cdUp();

	fbxRead( filename );
	fbxImport();
	fbxCleanup();

	bool allTransparent = true;
	for ( int i = 0; i < m_dsMaterials.size() && allTransparent; i++ )
	{
		if ( m_dsMaterials[i]->getBaseOpacity() > 0.1 )
		{
			allTransparent = false;
		}
	}

	if ( allTransparent )
	{
		for ( int i = 0; i < m_dsMaterials.size() && allTransparent; i++ )
		{
			m_dsMaterials[i]->setBaseOpacity( 1 );
		}
	}

	return DZ_NO_ERROR;
}

/**
**/
QString DzFbxImporter::getFileVersion() const
{
	QString sdkStr( "Unknown" );
	const QString versionStr( "%1 (%2.%3.%4)%5" );
	switch ( m_fbxFileMajor )
	{
	case 7:
		switch( m_fbxFileMinor )
		{
		case 7:
			sdkStr = "FBX 2019";
			break;
		case 5:
			sdkStr = "FBX 2016/2017";
			break;
		case 4:
			sdkStr = "FBX 2014/2015";
			break;
		case 3:
			sdkStr = "FBX 2013";
			break;
		case 2:
			sdkStr = "FBX 2012";
			break;
		case 1:
			sdkStr = "FBX 2011";
			break;
		default:
			break;
		}
		break;
	case 6:
		switch( m_fbxFileMinor )
		{
		case 1:
			sdkStr = "FBX 2006/2009/2010";
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return versionStr
		.arg( sdkStr )
		.arg( m_fbxFileMajor )
		.arg( m_fbxFileMinor )
		.arg( m_fbxFileRevision )
		.arg( getFileFormat() );
}

/**
**/
QString DzFbxImporter::getFileCreator() const
{
	return QString( m_fbxFileCreator );
}

/**
**/
QString DzFbxImporter::getFileFormat() const
{
	switch ( m_fbxFileBinary )
	{
	default:
	case -1:
		return QString();
	case 0:
		return " -- Ascii";
	case 1:
		return " -- Binary";
	}
}

/**
**/
QString DzFbxImporter::getSceneAuthor() const
{
	return QString( m_fbxSceneAuthor );
}

/**
**/
QString DzFbxImporter::getSceneTitle() const
{
	return QString( m_fbxSceneTitle );
}

/**
**/
QString DzFbxImporter::getSceneSubject() const
{
	return QString( m_fbxSceneSubject );
}

/**
**/
QString DzFbxImporter::getSceneKeywords() const
{
	return QString( m_fbxSceneKeywords );
}

/**
**/
QString DzFbxImporter::getSceneRevision() const
{
	return QString( m_fbxSceneRevision );
}

/**
**/
QString DzFbxImporter::getSceneComment() const
{
	return QString( m_fbxSceneComment );
}

/**
**/
QString DzFbxImporter::getOriginalAppVendor() const
{
	return QString( m_fbxOrigAppVendor );
}

/**
**/
QString DzFbxImporter::getOriginalAppName() const
{
	return QString( m_fbxOrigAppName );
}

/**
**/
QString DzFbxImporter::getOriginalAppVersion() const
{
	return QString( m_fbxOrigAppVersion );
}

/**
**/
QStringList DzFbxImporter::getAnimStackNames() const
{
	return m_animStackNames;
}

/**
**/
void DzFbxImporter::setRotationLimits( bool enable )
{
	m_includeRotationLimits = enable;
}

/**
**/
void DzFbxImporter::setIncludeAnimations( bool enable )
{
	m_includeAnimations = enable;
}

/**
**/
void DzFbxImporter::setTakeName( const QString &name )
{
	m_takeName = name;
}

/**
**/
void DzFbxImporter::setIncludePolygonSets( bool enable )
{
	m_includePolygonSets = enable;
}

/**
**/
void DzFbxImporter::setIncludePolygonGroups( bool enable )
{
	m_includePolygonGroups = enable;
}

/**
**/
void DzFbxImporter::setStudioNodeNamesLabels( bool enable )
{
	m_studioNodeNamesLabels = enable;
}

/**
**/
void DzFbxImporter::setStudioNodePresentation( bool enable )
{
	m_studioNodePresentation = enable;
}

/**
**/
void DzFbxImporter::setStudioNodeSelectionMap( bool enable )
{
	m_studioNodeSelectionMap = enable;
}

/**
**/
void DzFbxImporter::setStudioSceneIDs( bool enable )
{
	m_studioSceneIDs = enable;
}

/**
**/
QStringList DzFbxImporter::getErrorList() const
{
	return m_errorList;
}

/**
**/
void DzFbxImporter::replicateSkeleton( DzSkeleton* dsBaseSkeleton, const Skinning &skinning )
{
	Node* node = skinning.node;
	DzSkeleton* dsSkeleton = qobject_cast<DzSkeleton*>( node->dsNode );
	if ( !dsSkeleton )
	{
		return;
	}

	const Node* baseNode = m_root->find( dsBaseSkeleton );

	for ( int i = 0, n = baseNode->children.count(); i < n; i++ )
	{
		const Node* baseChild = baseNode->children.at( i );
		const FbxNodeAttribute* fbxBaseNodeAttr = baseChild->fbxNode->GetNodeAttribute();
		if ( fbxBaseNodeAttr && fbxBaseNodeAttr->GetAttributeType() == FbxNodeAttribute::eMesh )
		{
			continue;
		}

		Node* child = new Node();
		child->setParent( node );
		child->fbxNode = baseChild->fbxNode;
		child->dsParent = node->dsNode;

		fbxImportGraph( child );
	}

	dsSkeleton->setFollowTarget( dsBaseSkeleton );
}

static DzVec3 toVec3( FbxVector4 v )
{
	DzVec3 r;
	r.m_x = v[0];
	r.m_y = v[1];
	r.m_z = v[2];
	r.m_w = v[3];
	return r;
}

static FbxVector4 calcFbxRotationOffset( FbxNode* fbxNode )
{
	FbxVector4 offset( 0, 0, 0 );
	while ( fbxNode )
	{
		bool applyOffset = true;

		if ( fbxNode->GetNodeAttribute()
			&& fbxNode->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eMesh )
		{
			applyOffset = false;
		}

		if ( applyOffset )
		{
			offset += fbxNode->GetRotationOffset( FbxNode::eSourcePivot );
		}

		fbxNode = fbxNode->GetParent();
	}
	offset[3] = 1;
	return offset;
}

static void setNodeOrientation( DzNode* dsNode, FbxNode* fbxNode, DzFbxImporter::ECompatibilityMode m_compatibilityMode = DzFbxImporter::ECompatibilityMode::DefaultCompatibilityMode )
{
	FbxVector4 fbxPre = fbxNode->GetPreRotation( FbxNode::eSourcePivot );
	const DzQuat rot( DzRotationOrder::XYZ, DzVec3( fbxPre[0], fbxPre[1], fbxPre[2] ) * DZ_FLT_DEG_TO_RAD );

	dsNode->setOrientation( rot, true );

	// DB 2023-March-30: Support reading joint-orientation from other fields (PostRot, BindMatrix, etc)
	if (m_compatibilityMode)
	{
		// DB 2023-Mar-30: override pose and node transforms with bindmatrix from cluster
		FbxCluster* fbxCluster = FbxTools::findClusterFromNode(fbxNode);
		FbxAMatrix fbxBindMatrix;
		fbxBindMatrix.SetIdentity();
		if (fbxCluster)
		{
			fbxCluster->GetTransformLinkMatrix(fbxBindMatrix);
		}
		FbxAMatrix fbxPostMatrix;
		fbxPostMatrix.SetR(fbxNode->PostRotation.Get());
		FbxAMatrix fbxBindPostRotationMatrix = fbxBindMatrix * fbxPostMatrix;
		FbxVector4 fbxBindPostRotation = fbxBindPostRotationMatrix.GetR();
		const DzQuat newRot(DzRotationOrder::XYZ, DzVec3(fbxBindPostRotation[0], fbxBindPostRotation[1], fbxBindPostRotation[2]) * DZ_FLT_DEG_TO_RAD);
		dsNode->setOrientation(newRot, true);
	}

}

static void setNodeRotationOrder( DzNode* dsNode, FbxNode* fbxNode, DzFbxImporter::ECompatibilityMode m_compatibilityMode = DzFbxImporter::ECompatibilityMode::DefaultCompatibilityMode)
{
	//// DB 4-12-2023: compatibility for joint orientation up when no rotation order specified in file
	if (m_compatibilityMode)
	{
		if (fbxNode->RotationOrder.Get() == FbxEuler::eOrderXYZ &&
			fbxNode->PreRotation.Get() == FbxVector4(0, 0, 0))
		{
			FbxEuler::EOrder rotOrder = FbxEuler::eOrderYZX;
			fbxNode->RotationOrder.Set(rotOrder);
			fbxNode->SetRotationOrder(FbxNode::eSourcePivot, rotOrder);
		}
	}

	DzRotationOrder dsRotationOrder( DzRotationOrder::XYZ );
	EFbxRotationOrder fbxRotationOrder( eEulerXYZ );

	fbxNode->GetRotationOrder( FbxNode::eSourcePivot, fbxRotationOrder );
	switch ( fbxRotationOrder )
	{
	case eEulerXYZ:
		dsRotationOrder = DzRotationOrder::XYZ;
		break;
	case eEulerXZY:
		dsRotationOrder = DzRotationOrder::XZY;
		break;
	case eEulerYXZ:
		dsRotationOrder = DzRotationOrder::YXZ;
		break;
	case eEulerYZX:
		dsRotationOrder = DzRotationOrder::YZX;
		break;
	case eEulerZXY:
		dsRotationOrder = DzRotationOrder::ZXY;
		break;
	case eEulerZYX:
		dsRotationOrder = DzRotationOrder::ZYX;
		break;
	default:
		break;
	}

	dsNode->setRotationOrder( dsRotationOrder );
}

static void setNodeRotation( DzNode* dsNode, FbxNode* fbxNode, DzFbxImporter::ECompatibilityMode m_compatibilityMode = DzFbxImporter::ECompatibilityMode::DefaultCompatibilityMode)
{
	//const FbxDouble3 lclRotation = fbxNode->LclRotation.Get();
	FbxDouble3 lclRotation = fbxNode->LclRotation.Get();

	// DB 4-27-2023: Correction matrix for non-Daz poses
	// Daz Pose Control value is relative to the bind matrix, so we need to calculate the difference between the bind matrix and the
	//    current worldspace pose matrix but we also need to remove the parent contribution to bind matrix and pose matrix
	// Daz pose control = (parent inverse bind matrix * bind matrix).Inverse() * (parent inverse worldspace pose matrix * worldspace pose matrix )
	if (m_compatibilityMode && m_compatibilityMode != DzFbxImporter::ECompatibilityMode::DazStudioCompatibilityMode)
	{
		FbxAMatrix parentWSMatrix;
		parentWSMatrix.SetIdentity();
		FbxAMatrix parentBindMatrix;
		parentBindMatrix.SetIdentity();
		if (fbxNode->GetParent())
		{
			parentWSMatrix = fbxNode->GetParent()->EvaluateGlobalTransform();
			FbxTools::getBindMatrixFromCluster(fbxNode->GetParent(), parentBindMatrix);
		}
		FbxAMatrix currentWSMatrix = fbxNode->EvaluateGlobalTransform();
		FbxAMatrix bindMatrix;
		bindMatrix.SetIdentity();
		FbxTools::getBindMatrixFromCluster(fbxNode, bindMatrix);
		FbxAMatrix currentLclMatrix = parentWSMatrix.Inverse() * currentWSMatrix;
		FbxAMatrix localBindMatrix = parentBindMatrix.Inverse() * bindMatrix;
		FbxAMatrix lclMatrix = localBindMatrix.Inverse() * currentLclMatrix;

		FbxVector4 rotationVector = lclMatrix.GetR();
		float rotX = rotationVector[0];
		float rotY = rotationVector[1];
		float rotZ = rotationVector[2];
		lclRotation[0] = rotX;
		lclRotation[1] = rotY;
		lclRotation[2] = rotZ;

		dsNode->getXRotControl()->setDefaultValue( 0 );
		dsNode->getYRotControl()->setDefaultValue( 0 );
		dsNode->getZRotControl()->setDefaultValue( 0 );

		// DB 4-27-2023: TODO: add implementation for Max worldspace evaluation equation
		if (m_compatibilityMode == DzFbxImporter::ECompatibilityMode::MaxCompatibilityMode)
		{
			return;
		}
	}

	//dsNode->getXRotControl()->setDefaultValue( lclRotation[0] );
	//dsNode->getYRotControl()->setDefaultValue( lclRotation[1] );
	//dsNode->getZRotControl()->setDefaultValue( lclRotation[2] );
	dsNode->getXRotControl()->setValue( lclRotation[0] );
	dsNode->getYRotControl()->setValue( lclRotation[1] );
	dsNode->getZRotControl()->setValue( lclRotation[2] );
}

static void setNodeRotationLimits( DzNode* dsNode, FbxNode* fbxNode )
{
	const FbxLimits rotationLimits = fbxNode->GetRotationLimits();
	if ( !rotationLimits.GetActive() )
	{
		return;
	}

	const FbxDouble3 min = rotationLimits.GetMin();
	const FbxDouble3 max = rotationLimits.GetMax();

	if ( rotationLimits.GetMaxXActive()
		&& rotationLimits.GetMinXActive() )
	{
		dsNode->getXRotControl()->setIsClamped( true );
		dsNode->getXRotControl()->setMinMax( min[0], max[0] );
	}

	if ( rotationLimits.GetMaxYActive()
		&& rotationLimits.GetMinYActive() )
	{
		dsNode->getYRotControl()->setIsClamped( true );
		dsNode->getYRotControl()->setMinMax( min[1], max[1] );
	}

	if ( rotationLimits.GetMaxZActive()
		&& rotationLimits.GetMinZActive() )
	{
		dsNode->getZRotControl()->setIsClamped( true );
		dsNode->getZRotControl()->setMinMax( min[2], max[2] );
	}
}

static void setNodeTranslation( DzNode* dsNode, FbxNode* fbxNode, DzVec3 translationOffset, DzFbxImporter::ECompatibilityMode m_compatibilityMode = DzFbxImporter::ECompatibilityMode::DefaultCompatibilityMode)
{
	const FbxDouble3 translation = fbxNode->LclTranslation.Get();

	//const float posX = translation[0] - translationOffset[0];
	//const float posY = translation[1] - translationOffset[1];
	//const float posZ = translation[2] - translationOffset[2];
	float posX = translation[0] - translationOffset[0];
	float posY = translation[1] - translationOffset[1];
	float posZ = translation[2] - translationOffset[2];

	// DB 4-27-2023: Correction matrix for non-Daz poses
	// Daz Pose Control value is relative to the bind matrix, so we need to calculate the difference between the bind matrix and the
	//    current worldspace pose matrix but we also need to remove the parent contribution to bind matrix and pose matrix
	// Daz pose control = (parent inverse bind matrix * bind matrix).Inverse() * (parent inverse worldspace pose matrix * worldspace pose matrix )
	if (m_compatibilityMode && m_compatibilityMode != DzFbxImporter::ECompatibilityMode::DazStudioCompatibilityMode)
	{
		FbxVector4 lclTranslationVector;
		FbxAMatrix parentWSMatrix;
		parentWSMatrix.SetIdentity();
		FbxAMatrix parentBindMatrix;
		parentBindMatrix.SetIdentity();
		if (fbxNode->GetParent())
		{
			parentWSMatrix = fbxNode->GetParent()->EvaluateGlobalTransform();
			FbxTools::getBindMatrixFromCluster(fbxNode->GetParent(), parentBindMatrix);
		}
		FbxAMatrix currentWSMatrix = fbxNode->EvaluateGlobalTransform();
		FbxAMatrix bindMatrix;
		bindMatrix.SetIdentity();
		FbxTools::getBindMatrixFromCluster(fbxNode, bindMatrix);
		FbxAMatrix currentLclMatrix = parentWSMatrix.Inverse() * currentWSMatrix;
		FbxAMatrix localBindMatrix = parentBindMatrix.Inverse() * bindMatrix;
		FbxAMatrix lclMatrix = localBindMatrix.Inverse() * currentLclMatrix;

		lclTranslationVector = lclMatrix.GetT();
		float posX2 = lclTranslationVector[0];
		float posY2 = lclTranslationVector[1];
		float posZ2 = lclTranslationVector[2];
		posX = posX2;
		posY = posY2;
		posZ = posZ2;

		dsNode->getXPosControl()->setDefaultValue( 0 );
		dsNode->getYPosControl()->setDefaultValue( 0 );
		dsNode->getZPosControl()->setDefaultValue( 0 );

		// DB 4-27-2023: TODO: add implementation for Max worldspace evaluation equation
		if (m_compatibilityMode == DzFbxImporter::ECompatibilityMode::MaxCompatibilityMode)
		{
			return;
		}
	}

	//dsNode->getXPosControl()->setDefaultValue( posX );
	//dsNode->getYPosControl()->setDefaultValue( posY );
	//dsNode->getZPosControl()->setDefaultValue( posZ );
	dsNode->getXPosControl()->setValue( posX );
	dsNode->getYPosControl()->setValue( posY );
	dsNode->getZPosControl()->setValue( posZ );
}

static void setNodeInheritsScale( DzNode* dsNode, FbxNode* fbxNode )
{
	FbxTransform::EInheritType inheritType;
	fbxNode->GetTransformationInheritType( inheritType );
	dsNode->setInheritScale( inheritType != FbxTransform::eInheritRrs );
}

static void setNodeScaling( DzNode* dsNode, FbxNode* fbxNode )
{
	const FbxDouble3 scaling = fbxNode->LclScaling.Get();

	//dsNode->getXScaleControl()->setDefaultValue( scaling[0] );
	//dsNode->getYScaleControl()->setDefaultValue( scaling[1] );
	//dsNode->getZScaleControl()->setDefaultValue( scaling[2] );
	dsNode->getXScaleControl()->setValue( scaling[0] );
	dsNode->getYScaleControl()->setValue( scaling[1] );
	dsNode->getZScaleControl()->setValue( scaling[2] );
}

static void setNodeSceneID( DzNode* dsNode, FbxNode* fbxNode )
{
	QString dsSceneId;

#if FBXSDK_VERSION_MAJOR >= 2016
	const FbxProperty fbxSceneIdProperty = fbxNode->FindProperty( "StudioSceneID" );
	if ( fbxSceneIdProperty.IsValid() )
	{
		dsSceneId = QString( fbxSceneIdProperty.Get<FbxString>() );
	}
#endif

	if ( dsSceneId.isEmpty() )
	{
		return;
	}

	const DzUri nodeUri( dsSceneId );
	if ( nodeUri.getFilePath().isEmpty() )
	{
		return;
	}

	dsNode->modifyAsset( nodeUri ); //cause assetUri to be set
	dsNode->modifyAsset(); //cause assetUri to become the source and mark as modified
}

static void setNodePresentation( DzNode* dsNode, FbxNode* fbxNode )
{
	QString presentationType;
	QString autoFitBase;
	QString preferredBase;

#if FBXSDK_VERSION_MAJOR >= 2016
	const FbxProperty fbxPresentationTypeProperty = fbxNode->FindProperty( "StudioPresentationType" );
	if ( fbxPresentationTypeProperty.IsValid() )
	{
		presentationType = QString( fbxPresentationTypeProperty.Get<FbxString>() );
	}

	const FbxProperty fbxPresentationAutoFitBaseProperty = fbxNode->FindProperty( "StudioPresentationAutoFitBase" );
	if ( fbxPresentationAutoFitBaseProperty.IsValid() )
	{
		autoFitBase = QString( fbxPresentationAutoFitBaseProperty.Get<FbxString>() );
	}

	const FbxProperty fbxPresentationPreferredBaseProperty = fbxNode->FindProperty( "StudioPresentationPreferredBase" );
	if ( fbxPresentationPreferredBaseProperty.IsValid() )
	{
		preferredBase = QString( fbxPresentationPreferredBaseProperty.Get<FbxString>() );
	}
#endif

	if ( presentationType.isEmpty()
		&& autoFitBase.isEmpty()
		&& preferredBase.isEmpty() )
	{
		return;
	}

	DzPresentation* presentation = dsNode->getPresentation();
	if ( !presentation )
	{
		presentation = new DzPresentation();
		dsNode->setPresentation( presentation );
	}

	if ( !presentationType.isEmpty() )
	{
		presentation->setType( presentationType );
	}

	if ( !autoFitBase.isEmpty() )
	{
#if DZ_SDK_4_12_OR_GREATER
		presentation->setAutoFitBase( autoFitBase );
#else
		// DzPresentation::setAutoFitBase() is not in the 4.5 SDK, so we attempt
		// to use the meta-object to access/set the DzPresentation::autoFitBase
		// property - since 4.5.2.13.

		presentation->setProperty( "autoFitBase", autoFitBase );
#endif //DZ_SDK_4_12_OR_GREATER
	}

	if ( !preferredBase.isEmpty() )
	{
#if DZ_SDK_4_12_OR_GREATER
		presentation->setPreferredBase( preferredBase );
#else
		// DzPresentation::setPreferredBase() is not in the 4.5 SDK, so we attempt
		// to use the meta-object to access/set the DzPresentation::preferredBase
		// property - since 4.5.2.13.

		presentation->setProperty( "preferredBase", preferredBase );
#endif //DZ_SDK_4_12_OR_GREATER
	}
}

static bool isFuzzyClose( double a, double b, double c )
{
	if ( qAbs( a - b ) > 0.0000000001f
		|| qAbs( a - c ) > 0.0000000001f )
	{
		return false;
	}

	return true;
}

/**
**/
void DzFbxImporter::fbxPreImportAnimationStack()
{
	for ( int i = 0, n = m_fbxScene->GetSrcObjectCount<FbxAnimStack>(); i < n; i++ )
	{
		const FbxAnimStack* animStack = m_fbxScene->GetSrcObject<FbxAnimStack>( i );

		const QString stackName( animStack->GetName() );

		const int numLayers = animStack->GetMemberCount<FbxAnimLayer>();
		if ( numLayers == 0 )
		{
			m_errorList << "Animation: \"" % stackName % "\" has no layers.";
		}
		else if ( numLayers > 1 )
		{
			m_errorList << "Animation: \"" % stackName % "\" has multiple layers.";
		}

		m_animStackNames.push_back( stackName );
	}
}

/**
**/
void DzFbxImporter::fbxPreImportGraph( FbxNode* fbxNode )
{
	const QString nodeName( fbxNode->GetName() );

	if ( !m_suppressRigErrors )
	{
		// pre (Joint Orient)/post (Rotate Axis) rotation must match
		if ( fbxNode->GetPreRotation( FbxNode::eSourcePivot ) != fbxNode->GetPostRotation( FbxNode::eSourcePivot ) )
		{
			m_errorList << "Rigging: Pre/Post rotation mismatch: " % nodeName;
		}

		// scale must be uniform
		FbxDouble3 fbxScaling = fbxNode->LclScaling.Get();
		if ( !isFuzzyClose( fbxScaling[0], fbxScaling[1], fbxScaling[2] ) )
		{
			m_errorList << "Rigging: Non-uniform scale: " % nodeName;
		}

		// mesh
		if ( const FbxMesh* fbxMesh = fbxNode->GetMesh() )
		{
			// we are only concerned with skinning at the moment
			const int numSkins = fbxMesh->GetDeformerCount( FbxDeformer::eSkin );
			for ( int i = 0; i < numSkins; i++ )
			{
				// skinning weights must be linked to a bone
				//// DB 4-14-2023: bugfix for get skin deformer
				FbxSkin* fbxSkin = FbxCast<FbxSkin>( fbxMesh->GetDeformer( i, FbxDeformer::eSkin) );
				for ( int j = 0, m = fbxSkin->GetClusterCount(); j < m; j++ )
				{
					FbxNode* fbxClusterNode = fbxSkin->GetCluster( j )->GetLink();
					if ( fbxClusterNode && fbxClusterNode->GetSkeleton() )
					{
						continue;
					}

					m_errorList << "Rigging: Cluster linked to non-bone: " % QString( fbxClusterNode->GetName() );
				}
			}
		}

		// "bone chains"
		if ( const FbxSkeleton* fbxSkeleton = fbxNode->GetSkeleton() )
		{
			// a "bone chain" must ultimately start with a "root"; if the 'current'
			// skeleton node is not the root, it should have a parent that is

			const char* skelTypes[] = { "Root", "Limb", "Limb Node", "Effector" };

			FbxSkeleton::EType skelType = fbxSkeleton->GetSkeletonType();
			switch ( skelType )
			{
			default:
			case FbxSkeleton::eRoot:
				break;
			case FbxSkeleton::eLimb:
			case FbxSkeleton::eLimbNode:
			case FbxSkeleton::eEffector:
				{
					FbxNode* fbxParentNode = fbxNode->GetParent();
					if ( !fbxParentNode
						|| !fbxParentNode->GetSkeleton() )
					{
						m_errorList << "Rigging: Bone chain type incorrect - " % QString( skelTypes[skelType] ) % ": " % nodeName;
					}
				}
				break;
			}
		}
	}

	for ( int i = 0, n = fbxNode->GetChildCount(); i < n; i++ )
	{
		fbxPreImportGraph( fbxNode->GetChild( i ) );
	}
}

/**
**/
void DzFbxImporter::fbxPreImport()
{
	fbxPreImportAnimationStack();

	FbxNode* fbxRootNode = m_fbxScene->GetRootNode();
	for ( int i = 0, n = fbxRootNode->GetChildCount(); i < n; i++ )
	{
		fbxPreImportGraph( fbxRootNode->GetChild( i ) );
	}
}

/**
**/
void DzFbxImporter::fbxImportGraph( Node* node )
{
	if ( node == m_root )
	{
		for ( int i = 0; i < node->fbxNode->GetChildCount(); i++ )
		{
			Node* child = new Node();
			child->setParent( node );
			child->fbxNode = node->fbxNode->GetChild( i );
		}

		for ( int i = 0; i < node->children.count(); i++ )
		{
			fbxImportGraph( node->children[i] );
		}

		return;
	}

	DzNode* dsMeshNode = NULL;

	const FbxNull* fbxNull = node->fbxNode->GetNull();
	if ( fbxNull || !node->fbxNode->GetNodeAttribute() )
	{
		if (FbxTools::checkIfChildrenAreBones(node->fbxNode))
		{
			node->dsNode = createFigure();
		}
		else
		{
			node->dsNode = new DzNode();
		}
	}
	else
	{
		switch ( node->fbxNode->GetNodeAttribute()->GetAttributeType() )
		{
		case FbxNodeAttribute::eMarker:
			break;
		case FbxNodeAttribute::eSkeleton:
			{
				const FbxSkeleton* fbxSkeleton = node->fbxNode->GetSkeleton();
				switch ( fbxSkeleton->GetSkeletonType() )
				{
				case FbxSkeleton::eRoot:
					node->dsNode = createFigure();
					// DB 4-20-2023: follow-parent fix
					if (m_nodeMap.find(node->fbxNode->GetParent()) != m_nodeMap.end())
					{
						DzNode* dsParent = m_nodeMap[node->fbxNode->GetParent()];
						DzFigure* dsParentFigure = qobject_cast<DzFigure*>(dsParent);
						DzFigure* dsFigure = qobject_cast<DzFigure*>(node->dsNode);
						if (dsFigure && dsParentFigure)
						{
							dsFigure->setFollowTarget(dsParent->getSkeleton());
							DzEnumProperty* followModeControl = NULL;
							if (!QMetaObject::invokeMethod(dsFigure,
								"getFollowModeControl", Q_RETURN_ARG(DzEnumProperty*, followModeControl))
								|| !followModeControl)
							{
								followModeControl = qobject_cast<DzEnumProperty*>(dsFigure->findProperty("Fit to Mode"));
							}
							if (followModeControl)
							{
								followModeControl->setValue(1);
							}
						}
					}
					break;
				case FbxSkeleton::eLimb: //intentional fall-through
				case FbxSkeleton::eLimbNode:
					{
						node->dsNode = new DzBone();
						node->dsNode->setInheritScale( true );

#if FBXSDK_VERSION_MAJOR >= 2016
						FbxProperty fbxPropertyFaceGroup;
						if ( m_studioNodeSelectionMap )
						{
							fbxPropertyFaceGroup = node->fbxNode->FindProperty( "StudioNodeFaceGroup" );
						}

						if ( fbxPropertyFaceGroup.IsValid() )
						{
							const QString selectionSetName( fbxPropertyFaceGroup.Get<FbxString>() );
							m_nodeFaceGroupMap.insert( node, selectionSetName );
						}
#endif
					}
					break;
				case FbxSkeleton::eEffector:
					break;
				}
			}
			break;
		case FbxNodeAttribute::eMesh:
			{
				const QString fbxNodeName( node->fbxNode->GetName() );
				if ( node->dsParent &&
					!node->dsParent->getObject() &&
					( node->dsParent->getName() + ".Shape" == fbxNodeName ||
						node->dsParent->getName() + "_Shape" == fbxNodeName ) )
				{
					dsMeshNode = node->dsParent;
				}

				// DB 2023-Mar-31: Find skeleton root/parent node for mesh node
				if (m_compatibilityMode)
				{
					if (!dsMeshNode)
					{
						FbxNode* fbxParent = nullptr;
						fbxParent = FbxTools::findAssociatedSkeletonRoot(node->fbxNode->GetMesh());
						if (FbxTools::isBone(fbxParent, FbxSkeleton::EType::eLimb) ||
							FbxTools::isBone(fbxParent, FbxSkeleton::EType::eLimbNode))
						{
							fbxParent = fbxParent->GetParent();
						}
						if (fbxParent && m_nodeMap.find(fbxParent) != m_nodeMap.end())
						{
							auto debugFbxParentName = fbxParent->GetName();
							DzNode* dsParent = m_nodeMap[fbxParent];
							if (dsParent)
							{
								auto debugObj = dsParent->getObject();
								auto debugName = dsParent->getName();
								printf("nop");
							}
							if (dsParent &&
								!dsParent->getObject() &&
								(dsParent->getName() + ".Shape" == fbxNodeName ||
									dsParent->getName() + "_Shape" == fbxNodeName))
							{
								dsMeshNode = dsParent;
							}
						}
					}
				}

				if (!dsMeshNode)
				{
					const FbxMesh* fbxMesh = node->fbxNode->GetMesh();
					if ( fbxMesh->GetDeformerCount( FbxDeformer::eSkin ) > 0 )
					{
						node->dsNode = createFigure();
						node->collapseTranslation = true;
						// DB 4-20-2023: follow-parent fix
						if (m_nodeMap.find(node->fbxNode->GetParent()) != m_nodeMap.end())
						{
							DzNode* dsParent = m_nodeMap[node->fbxNode->GetParent()];
							DzFigure* dsParentFigure = qobject_cast<DzFigure*>(dsParent);
							DzFigure* dsFigure = qobject_cast<DzFigure*>(node->dsNode);
							if (dsFigure && dsParentFigure)
							{
								QString parentLabel = dsParent->getLabel();
								dsFigure->setFollowTarget(dsParent->getSkeleton());
							}
						}
					}
					else
					{
						node->dsNode = new DzNode();
					}

					dsMeshNode = node->dsNode;
				}

				fbxImportMesh( node, node->fbxNode, dsMeshNode );
			}
			break;
		case FbxNodeAttribute::eNurbs:
			break;
		case FbxNodeAttribute::ePatch:
			break;
		case FbxNodeAttribute::eCamera:
			break;
		case FbxNodeAttribute::eLight:
			break;
		case FbxNodeAttribute::eLODGroup:
			break;
		default:
			FbxNodeAttribute::EType type = node->fbxNode->GetNodeAttribute()->GetAttributeType();
			break;
		}
	}

	if ( node->dsNode )
	{
		m_nodeMap[node->fbxNode] = node->dsNode;

		QString nodeName( node->fbxNode->GetName() );
#if FBXSDK_VERSION_MAJOR >= 2016
		FbxProperty fbxPropertyNodeName;
		if ( m_studioNodeNamesLabels )
		{
			fbxPropertyNodeName = node->fbxNode->FindProperty( "StudioNodeName" );
		}

		if ( fbxPropertyNodeName.IsValid() )
		{
			nodeName = fbxPropertyNodeName.Get<FbxString>();
		}
#endif
		node->dsNode->setName( nodeName );

		if ( node->dsParent )
		{
			node->dsParent->addNodeChild( node->dsNode );
		}

#if FBXSDK_VERSION_MAJOR >= 2016
		FbxProperty fbxPropertyNodeLabel;
		if ( m_studioNodeNamesLabels )
		{
			fbxPropertyNodeLabel = node->fbxNode->FindProperty( "StudioNodeLabel" );
		}

		if ( fbxPropertyNodeLabel.IsValid() )
		{
			const QString nodeLabel( fbxPropertyNodeLabel.Get<FbxString>());
			node->dsNode->setLabel( dzScene->getUniqueTopLevelLabel( nodeLabel ) );
		}
#endif

		if ( m_studioNodePresentation )
		{
			setNodePresentation( node->dsNode, node->fbxNode );
		}

		setNodeInheritsScale( node->dsNode, node->fbxNode );

		const FbxVector4 rotationOffset = calcFbxRotationOffset( node->fbxNode );
		node->dsNode->setOrigin(toVec3(rotationOffset), true);
		setNodeOrientation(node->dsNode, node->fbxNode, m_compatibilityMode);
		setNodeRotationOrder( node->dsNode, node->fbxNode, m_compatibilityMode );

		if ( rotationOffset.SquareLength() == 0.0 )
		{
			bool found = false;
			FbxMatrix fbxMatrix;

			for ( int i = 0; i < m_fbxScene->GetPoseCount(); i++ )
			{
				const FbxPose* pose = m_fbxScene->GetPose( i );
				if ( pose->IsBindPose() )
				{
					for ( int j = 0; j < pose->GetCount(); j++ )
					{
						if ( pose->GetNode( j ) == node->fbxNode )
						{
							found = true;
							fbxMatrix = pose->GetMatrix( j );
						}
					}
				}
			}

			if ( !found )
			{
				fbxMatrix = node->fbxNode->EvaluateGlobalTransform();
			}

			node->bindTranslation[0] = fbxMatrix[3][0];
			node->bindTranslation[1] = fbxMatrix[3][1];
			node->bindTranslation[2] = fbxMatrix[3][2];
			if ( !node->collapseTranslation )
			{
				node->dsNode->setOrigin( node->bindTranslation, true );
			}
		}

		if (m_compatibilityMode)
		{
			FbxVector4 nodePosition = rotationOffset;
			// DB 2023-Mar-30: override pose and node transforms with bindmatrix from cluster
			FbxCluster* fbxCluster = FbxTools::findClusterFromNode(node->fbxNode);
			FbxAMatrix fbxBindMatrix;
			fbxBindMatrix.SetIdentity();
			if (fbxCluster)
			{
				fbxCluster->GetTransformLinkMatrix(fbxBindMatrix);
				FbxVector4 fbxBindPosition = fbxBindMatrix.GetT();
				fbxBindPosition[3] = 1.0;
				nodePosition = fbxBindPosition;
				if (rotationOffset.SquareLength() == 0.0)
				{
					node->bindTranslation[0] = fbxBindPosition[0];
					node->bindTranslation[1] = fbxBindPosition[1];
					node->bindTranslation[2] = fbxBindPosition[2];
				}
			}
			node->dsNode->setOrigin(toVec3(nodePosition), true);

			//FbxAMatrix fbxPostMatrix;
			//fbxPostMatrix.SetR(node->fbxNode->PostRotation.Get());
			//FbxAMatrix fbxBindPostRotationMatrix = fbxBindMatrix * fbxPostMatrix;
			//FbxVector4 fbxBindPostRotation = fbxBindPostRotationMatrix.GetR();
			//const DzQuat newRot(DzRotationOrder::XYZ, DzVec3(fbxBindPostRotation[0], fbxBindPostRotation[1], fbxBindPostRotation[2]) * DZ_FLT_DEG_TO_RAD);
			//node->dsNode->setOrientation(newRot, true);
		}

		if ( node->collapseTranslation && dsMeshNode )
		{
			DzFacetMesh* dsFacetMesh = qobject_cast<DzFacetMesh*>( dsMeshNode->getObject()->getCurrentShape()->getGeometry() );
			if ( dsFacetMesh )
			{
				DzPnt3* vertices = dsFacetMesh->getVerticesPtr();
				for ( int i = 0; i < dsFacetMesh->getNumVertices(); i++ )
				{
					vertices[i][0] += node->bindTranslation[0];
					vertices[i][1] += node->bindTranslation[1];
					vertices[i][2] += node->bindTranslation[2];
				}
			}
		}

		dzScene->addNode( node->dsNode );

#if DZ_SDK_4_12_OR_GREATER
		addImportedNode( node->dsNode );
#endif

		for ( int i = 0; i < node->fbxNode->GetChildCount(); i++ )
		{
			Node* child = new Node();
			child->setParent( node );
			child->dsParent = node->dsNode;
			child->fbxNode = node->fbxNode->GetChild( i );
		}

		for ( int i = 0; i < node->children.count(); i++ )
		{
			fbxImportGraph( node->children[i] );
		}

		DzVec3 originPoint = node->dsNode->getOrigin();
		DzVec3 endPoint = node->dsNode->getOrigin();
		if ( node->dsNode->getNumNodeChildren() )
		{
			endPoint = node->dsNode->getNodeChild( 0 )->getOrigin( true );
			for ( int i = 1; i < node->dsNode->getNumNodeChildren(); i++ )
			{
				endPoint += node->dsNode->getNodeChild( i )->getOrigin( true );
			}
			endPoint.m_x /= node->dsNode->getNumNodeChildren();
			endPoint.m_y /= node->dsNode->getNumNodeChildren();
			endPoint.m_z /= node->dsNode->getNumNodeChildren();
			node->dsNode->setEndPoint( endPoint, true );
			//// DB 4-12-2023: endPoint testing
//			DzVec3 delta = endPoint - originPoint;
//			node->dsNode->setEndPoint(originPoint + delta*0.3, true);
		}
		else
		{
			DzVec3 toCenter = endPoint;
			if ( node->dsNode->getNodeParent() )
			{
				toCenter -= node->dsNode->getNodeParent()->getOrigin();
				toCenter = ( node->dsNode->getNodeParent()->getOrientation().inverse() * node->dsNode->getOrientation() ).multVec( toCenter );
				if ( toCenter.length() > 1 )
				{
					toCenter /= 2.0;
				}
			}
			node->dsNode->setEndPoint( endPoint + toCenter, true );
		}

		// setup a decent guess so IK will work
		if ( qobject_cast<DzBone*>( node->dsNode ) &&
			qobject_cast<DzBone*>( node->dsParent ) &&
			node->dsNode->getNumNodeChildren() != 0 )
		{
			node->dsNode->getXPosControl()->setHidden( true );
			node->dsNode->getYPosControl()->setHidden( true );
			node->dsNode->getZPosControl()->setHidden( true );
		}

		if ( m_includeRotationLimits )
		{
			setNodeRotationLimits( node->dsNode, node->fbxNode );
		}

		if ( m_studioSceneIDs )
		{
			setNodeSceneID( node->dsNode, node->fbxNode );
		}
	}
}

/**
**/
void DzFbxImporter::fbxImportAnimation( Node* node )
{
	// DB 4-27-23: Fix for recursively double-dipped posing of follower skeletons
	// if node is following another skeleton, then skip
	if (node->dsNode->getSkeleton() != NULL && node->dsNode->getSkeleton()->getFollowTarget() != NULL)
	{
		return;
	}

	if ( node->dsNode )
	{
		if ( !node->collapseTranslation )
		{
			DzVec3 translationOffset = node->bindTranslation;
			if ( node->parent )
			{
				translationOffset -= node->parent->bindTranslation;
			}

			//if (getOriginalAppName().compare("Daz Studio", Qt::CaseInsensitive)!=0 && m_compatibilityMode != DzFbxImporter::ECompatibilityMode::DefaultCompatibilityMode)
			if (m_compatibilityMode != DzFbxImporter::ECompatibilityMode::DefaultCompatibilityMode)
			{
				DzFbxImporter::ECompatibilityMode overrideCompatibilityMode = m_compatibilityMode;
				if (getOriginalAppName().compare("Daz Studio", Qt::CaseInsensitive) == 0)
				{
					overrideCompatibilityMode = DzFbxImporter::ECompatibilityMode::DazStudioCompatibilityMode;
				}
				else if (getOriginalAppName().compare("3ds Max", Qt::CaseInsensitive) == 0)
				{
					overrideCompatibilityMode = DzFbxImporter::ECompatibilityMode::MaxCompatibilityMode;
				}
				// DB 4-27-23: Fix for recursively double-dipped posing of follower skeletons
				// if node is following another skeleton, then skip
				bool bSkipPose = false;
				if (node->dsNode->getSkeleton() != NULL && node->dsNode->getSkeleton()->getFollowTarget() != NULL)
				{
					bSkipPose = true;
				}
				if (bSkipPose == false)
				{
					setNodeTranslation(node->dsNode, node->fbxNode, translationOffset, overrideCompatibilityMode);
					setNodeRotation(node->dsNode, node->fbxNode, overrideCompatibilityMode);
					setNodeScaling(node->dsNode, node->fbxNode);
				}
			}
			else
			{
				setNodeTranslation(node->dsNode, node->fbxNode, translationOffset);
				setNodeRotation(node->dsNode, node->fbxNode);
				setNodeScaling(node->dsNode, node->fbxNode);
			}
		}

		if ( m_fbxAnimLayer && !node->collapseTranslation )
		{
			// DB 4-27-2023: TODO: To support non-Daz Studio animations: modify all curves to remove parent bindmatrix offset from children bindmatrix
			//  See correction matrix creation and application in setNodeTranslation() and setNodeRotation()

			applyFbxCurve( node->fbxNode->LclTranslation.GetCurve( m_fbxAnimLayer, FBXSDK_CURVENODE_COMPONENT_X ), node->dsNode->getXPosControl() );
			applyFbxCurve( node->fbxNode->LclTranslation.GetCurve( m_fbxAnimLayer, FBXSDK_CURVENODE_COMPONENT_Y ), node->dsNode->getYPosControl() );
			applyFbxCurve( node->fbxNode->LclTranslation.GetCurve( m_fbxAnimLayer, FBXSDK_CURVENODE_COMPONENT_Z ), node->dsNode->getZPosControl() );

			applyFbxCurve( node->fbxNode->LclRotation.GetCurve( m_fbxAnimLayer, FBXSDK_CURVENODE_COMPONENT_X ), node->dsNode->getXRotControl() );
			applyFbxCurve( node->fbxNode->LclRotation.GetCurve( m_fbxAnimLayer, FBXSDK_CURVENODE_COMPONENT_Y ), node->dsNode->getYRotControl() );
			applyFbxCurve( node->fbxNode->LclRotation.GetCurve( m_fbxAnimLayer, FBXSDK_CURVENODE_COMPONENT_Z ), node->dsNode->getZRotControl() );

			applyFbxCurve( node->fbxNode->LclScaling.GetCurve( m_fbxAnimLayer, FBXSDK_CURVENODE_COMPONENT_X ), node->dsNode->getXScaleControl() );
			applyFbxCurve( node->fbxNode->LclScaling.GetCurve( m_fbxAnimLayer, FBXSDK_CURVENODE_COMPONENT_Y ), node->dsNode->getYScaleControl() );
			applyFbxCurve( node->fbxNode->LclScaling.GetCurve( m_fbxAnimLayer, FBXSDK_CURVENODE_COMPONENT_Z ), node->dsNode->getZScaleControl() );
		}


#if FBXSDK_VERSION_MAJOR >= 2016
		for ( FbxProperty fbxProperty = node->fbxNode->GetFirstProperty(); fbxProperty.IsValid(); fbxProperty = node->fbxNode->GetNextProperty( fbxProperty ) )
		{
			if ( !fbxProperty.GetFlag( FbxPropertyFlags::eUserDefined ) )
			{
				continue;
			}

			if ( node->dsNode->findDataItem( DATA_FBX_USER_PROPERTIES ) == NULL )
			{
				node->dsNode->addDataItem( new DzSimpleElementData( DATA_FBX_USER_PROPERTIES, true ) );
			}

			const QString key( fbxProperty.GetName() );
			DzSimpleElementData* userPropertyData = qobject_cast<DzSimpleElementData*>( node->dsNode->findDataItem( DATA_FBX_USER_PROPERTIES ) );
			DzSettings* userSettings = userPropertyData->getSettings();

			FbxDataType fbxPropertyData = fbxProperty.GetPropertyDataType();
			switch ( fbxPropertyData.GetType() )
			{
			case eFbxInt:
				userSettings->setIntValue( key, fbxProperty.Get<int>() );
				break;
			case eFbxBool:
				userSettings->setBoolValue( key, fbxProperty.Get<bool>() );
				break;
			case eFbxFloat:
				userSettings->setFloatValue( key, fbxProperty.Get<float>() );
				break;
			case eFbxDouble:
				userSettings->setFloatValue( key, fbxProperty.Get<double>() );
				break;
			case eFbxString:
				userSettings->setStringValue( key, QString( fbxProperty.Get<FbxString>() ) );
				break;
			default:
				break;
			}
		}
#endif
	}

	for ( int i = 0; i < node->children.count(); i++ )
	{
		fbxImportAnimation( node->children[i] );
	}
}


static QColor toQColor( FbxPropertyT<FbxDouble3> fbxValue )
{
	QColor clr;

	clr.setRedF( fbxValue.Get()[0] );
	clr.setGreenF( fbxValue.Get()[1] );
	clr.setBlueF( fbxValue.Get()[2] );

	return clr;
}

/**
**/
DzTexture* DzFbxImporter::toTexture( FbxProperty fbxProperty )
{
	for ( int i = 0; i < fbxProperty.GetSrcObjectCount<FbxFileTexture>(); ++i )
	{
		const FbxFileTexture* fbxFileTexture = fbxProperty.GetSrcObject<FbxFileTexture>( i );
		const DzImageMgr* imgMgr = dzApp->getImageMgr();
		DzTexture* dsTexture = imgMgr->getImage( fbxFileTexture->GetFileName() );
		if ( !dsTexture )
		{
			dsTexture = imgMgr->getImage( m_folder.filePath( fbxFileTexture->GetFileName() ) );
		}

		return dsTexture;
	}

	return 0;
}

/**
**/
void DzFbxImporter::fbxImportVertices( int numVertices, FbxVector4* fbxVertices, DzFacetMesh* dsMesh, DzVec3 offset )
{
	DzPnt3* dsVertices = dsMesh->setVertexArray( numVertices );
	for ( int i = 0; i < numVertices; i++ )
	{
		if (m_compatibilityMode)
		{
			fbxVertices[i] = m_axisCorrectionMatrix.MultT(fbxVertices[i]);
		}
		dsVertices[i][0] = fbxVertices[i][0] + offset[0];
		dsVertices[i][1] = fbxVertices[i][1] + offset[1];
		dsVertices[i][2] = fbxVertices[i][2] + offset[2];
	}
}

/**
**/
void DzFbxImporter::fbxImportUVs( FbxMesh* fbxMesh, DzFacetMesh* dsMesh )
{
	for ( int i = 0, n = fbxMesh->GetElementUVCount(); i < n; i++ )
	{
		const FbxGeometryElementUV* fbxGeomUv = fbxMesh->GetElementUV( i );
		const int numUvs = fbxGeomUv->GetDirectArray().GetCount();

		DzMap* dsUvMap = dsMesh->getUVs();
		dsUvMap->setNumValues( numUvs );
		DzPnt2* dsUVs = dsUvMap->getPnt2ArrayPtr();

		for ( int j = 0; j < numUvs; j++ )
		{
			const FbxVector2 fbxUv = fbxGeomUv->GetDirectArray().GetAt( j );
			dsUVs[j][0] = fbxUv[0];
			dsUVs[j][1] = fbxUv[1];
		}

		// only do the first
		break;
	}
}

/**
**/
void DzFbxImporter::fbxImportSubdVertexWeights( FbxMesh* fbxMesh, DzFacetMesh* dsMesh, bool &enableSubd )
{
	for ( int i = 0, n = fbxMesh->GetElementVertexCreaseCount(); i < n; i++ )
	{
		const FbxGeometryElementCrease* fbxSubdVertexCrease = fbxMesh->GetElementVertexCrease( i );
		for ( int j = 0, m = fbxSubdVertexCrease->GetDirectArray().GetCount(); j < m; j++ )
		{
			const double weight = fbxSubdVertexCrease->GetDirectArray().GetAt( j );
			if ( weight > 0 )
			{
				enableSubd = true;

#if DZ_SDK_4_12_OR_GREATER
				dsMesh->setVertexWeight( j, weight );
#else
				// DzFacetMesh::setVertexWeight() is not in the 4.5 SDK, so we
				// attempt to use the meta-object to call the method.

				bool im = QMetaObject::invokeMethod( dsMesh, "setVertexWeight",
					Q_ARG( int, j ), Q_ARG( int, weight ) );
				assert( im );
#endif
			}
		}

		// only do the first
		break;
	}
}

/**
**/
void DzFbxImporter::fbxImportMaterials( FbxNode* fbxNode, FbxMesh* fbxMesh, DzFacetMesh* dsMesh, DzFacetShape* dsShape, bool &matsAllSame )
{
	for ( int i = 0, n = fbxNode->GetMaterialCount(); i < n; i++ )
	{
		QColor diffuseColor = Qt::white;
		DzTexturePtr diffuseMap = NULL;

		float diffuseFactor = 1.0f;

		float opacityBase = 1.0f;
		DzTexturePtr opacityMap = NULL;

		QColor ambientColor = Qt::black;
		DzTexturePtr ambientMap = NULL;

		float ambientFactor = 1.0f;

		QColor specularColor = Qt::white;
		DzTexturePtr specularMap = NULL;

		float specularFactor = 1.0f;

		float shininess = 1.0f;
		DzTexturePtr shininessMap = NULL;

		float reflectionFactor = 1.0f;
		DzTexturePtr reflectionMap = NULL;

		float roughness = 0.1f;
		float metallicity = 1.0f;

		FbxSurfaceMaterial* fbxMaterial = fbxNode->GetMaterial( i );
		DzMaterialPtr dsMaterial = NULL;

		const DzClassFactory* pbrMatFactory = dzApp->findClassFactory( "DzPbrMaterial" ); //or "DzUberIrayMaterial"
		if ( QObject* pbrMatInstance = pbrMatFactory ? pbrMatFactory->createInstance() : NULL )
		{
			dsMaterial = qobject_cast<DzMaterial*>( pbrMatInstance );
		}
		else
		{
			dsMaterial = new DzDefaultMaterial();
		}

		DzDefaultMaterial* dsDefMaterial = qobject_cast<DzDefaultMaterial*>( dsMaterial );

		FbxSurfacePhong* fbxPhong = FbxCast<FbxSurfacePhong>( fbxMaterial );
		if ( fbxPhong )
		{
			diffuseColor = toQColor( fbxPhong->Diffuse );
			diffuseMap = toTexture( fbxPhong->Diffuse );

			// Maya and Max want transparency in the color
			opacityBase = 1 - (fbxPhong->TransparentColor.Get()[0] + fbxPhong->TransparentColor.Get()[1] + fbxPhong->TransparentColor.Get()[2]) / 3;
			opacityMap = toTexture( fbxPhong->TransparentColor );
			// DB: 4-25-23: Phong Transparency Fix for Maya and Daz Studio
			if (m_compatibilityMode)
			{
				FbxDouble fbxTransparencyFactor = fbxPhong->TransparencyFactor.Get();
				opacityBase = 1 - (fbxPhong->TransparentColor.Get()[0] * fbxTransparencyFactor + fbxPhong->TransparentColor.Get()[1] * fbxTransparencyFactor + fbxPhong->TransparentColor.Get()[2] * fbxTransparencyFactor) / 3;
				if (opacityBase != 1.0f && getOriginalAppName().compare("Daz Studio", Qt::CaseInsensitive) == 0)
				{
					opacityBase = 1 - opacityBase;
				}
			}

			if ( dsDefMaterial )
			{
				diffuseFactor = fbxPhong->DiffuseFactor.Get();

				ambientColor = toQColor( fbxPhong->Ambient );
				ambientMap = toTexture( fbxPhong->Ambient );

				ambientFactor = fbxPhong->AmbientFactor.Get();

				specularColor = toQColor( fbxPhong->Specular );
				specularMap = toTexture( fbxPhong->Specular );

				specularFactor = fbxPhong->SpecularFactor.Get();

				shininess = fbxPhong->Shininess.Get();
				shininessMap = toTexture( fbxPhong->Shininess );

				reflectionFactor = fbxPhong->ReflectionFactor.Get();
				reflectionMap = toTexture( fbxPhong->ReflectionFactor );
			}
			else //DzPbrMaterial or DzUberIrayMaterial
			{
				roughness = 1.0 - ((log( fbxPhong->Shininess.Get() ) / log( 2.0 )) - 2) / 10;

				FbxDouble3 spec = fbxPhong->Specular.Get();
				float innerDistance = qAbs( spec[1] - spec[0] ) + qAbs( spec[2] - spec[0] ) + qAbs( spec[2] - spec[1] );
				metallicity = qMin( 1.0f, innerDistance );
			}
		}
		else if ( FbxSurfaceLambert* fbxLambert = FbxCast<FbxSurfaceLambert>( fbxMaterial ) )
		{
			diffuseColor = toQColor( fbxLambert->Diffuse );
			diffuseMap = toTexture( fbxLambert->Diffuse );

			// Maya and Max want transparency in the color
			opacityBase = 1 - (fbxLambert->TransparentColor.Get()[0] + fbxLambert->TransparentColor.Get()[1] + fbxLambert->TransparentColor.Get()[2]) / 3;
			opacityMap = toTexture( fbxLambert->TransparentColor );

			if ( dsDefMaterial )
			{
				ambientColor = toQColor( fbxLambert->Ambient );
				ambientMap = toTexture( fbxLambert->Ambient );

				ambientFactor = fbxLambert->AmbientFactor.Get();
			}
		}

		dsMaterial->setName( fbxMaterial->GetName() );

		dsMaterial->setDiffuseColor( diffuseColor );
		dsMaterial->setColorMap( diffuseMap );

		dsMaterial->setBaseOpacity( opacityBase );
		dsMaterial->setOpacityMap( opacityMap );

		if ( dsDefMaterial )
		{
			dsDefMaterial->setAmbientColor( ambientColor );
			dsDefMaterial->setAmbientColorMap( ambientMap );

			dsDefMaterial->setAmbientStrength( ambientFactor );

			if ( fbxPhong )
			{
				dsDefMaterial->setDiffuseStrength( diffuseFactor );

				dsDefMaterial->setSpecularColor( specularColor );
				dsDefMaterial->setSpecularColorMap( specularMap );

				dsDefMaterial->setSpecularStrength( specularFactor );

				dsDefMaterial->setGlossinessStrength( shininess );
				dsDefMaterial->setGlossinessValueMap( shininessMap );

				dsDefMaterial->setReflectionStrength( reflectionFactor );
				dsDefMaterial->setReflectionMap( reflectionMap );
			}
		}
		else if ( fbxPhong ) //DzPbrMaterial or DzUberIrayMaterial
		{
			// Because DzPbrMaterial is not in the public SDK, we attempt to use
			// the meta-object to call the methods. If this fails, we attempt to
			// find the properties by name and if found set their respective values.

			// use "setGlossyRoughness" double if using DzUberIrayMaterial
			if ( !QMetaObject::invokeMethod( dsMaterial,
				"setRoughness", Q_ARG( float, roughness ) ) )
			{
				if ( DzFloatProperty* fProp = qobject_cast<DzFloatProperty*>( dsMaterial->findProperty( "Glossy Roughness" ) ) )
				{
					fProp->setValue( roughness );
				}
			}

			//use "setMetallicity" double if using DzUberIrayMaterial
			if ( !QMetaObject::invokeMethod( dsMaterial,
				"setMetallicity", Q_ARG( float, metallicity ) ) )
			{
				if ( DzFloatProperty* fProp = qobject_cast<DzFloatProperty*>( dsMaterial->findProperty( "Metallic Weight" ) ) )
				{
					fProp->setValue( metallicity );
				}
			}
		}

		m_dsMaterials.push_back( dsMaterial );

		dsShape->addMaterial( dsMaterial );
		dsMesh->activateMaterial( dsMaterial->getName() );
	}

	matsAllSame = true;
	for ( int i = 0, n = fbxMesh->GetElementMaterialCount(); i < n; i++ )
	{
		const FbxGeometryElementMaterial* fbxMaterial = fbxMesh->GetElementMaterial( i );
		if ( fbxMaterial->GetMappingMode() == FbxGeometryElement::eByPolygon )
		{
			matsAllSame = false;
			break;
		}
	}

	if ( matsAllSame )
	{
		for ( int i = 0, n = fbxMesh->GetElementMaterialCount(); i < n; i++ )
		{
			const FbxGeometryElementMaterial* fbxMaterial = fbxMesh->GetElementMaterial( i );
			if ( fbxMaterial->GetMappingMode() == FbxGeometryElement::eAllSame )
			{
				const int matIdx = fbxMaterial->GetIndexArray().GetAt( 0 );
				if ( matIdx >= 0 )
				{
					dsMesh->activateMaterial( matIdx );
					break;
				}
			}
		}
	}
}

#if !DZ_SDK_4_12_OR_GREATER
static void selectFacetsByIndexList( DzFacetMesh* dsMesh, DzFaceGroup* dsFaceGroup )
{
	const int nFacets = dsMesh->getNumFacets();
	const int* faceGrpIndices = dsFaceGroup->getIndicesPtr();
	unsigned char* facetFlags = dsMesh->getFacetFlagsPtr();
	for ( int i = 0, n = dsFaceGroup->count(); i < n; ++i )
	{
		const int faceGrpIdx = faceGrpIndices[i];
		if ( faceGrpIdx >= nFacets )
		{
			break;
		}

		if ( facetFlags[faceGrpIdx] & DZ_HIDDEN_FACE_BIT )
		{
			continue;
		}

		facetFlags[faceGrpIdx] |= DZ_SELECTED_FACE_BIT;
	}
}
#endif

/**
	Builds face groups, or polygon selection sets, from polygon selection set
	data in the FBX.

	@param dsMeshNode	The node that provides the mesh with the polygons we are
						interested in. Used to validate that a given selection
						set is intended for 'this' object.
	@param dsMesh		The facet mesh that we will alter the face group(s) of.
	@param dsShape		The shape to create a facet selection group on; depending
						on the active options.

	@sa fbxImportMesh()
**/
void DzFbxImporter::fbxImportPolygonSets( DzNode* dsMeshNode, DzFacetMesh* dsMesh, DzFacetShape* dsShape )
{
	if ( !m_includePolygonSets )
	{
		return;
	}

	const bool asFaceGroups = !m_includePolygonGroups;

	for ( int i = 0, n = m_fbxScene->GetMemberCount<FbxSelectionSet>(); i < n; i++ )
	{
		FbxSelectionSet* fbxSelectionSet = m_fbxScene->GetMember<FbxSelectionSet>( i );
		if ( !fbxSelectionSet )
		{
			continue;
		}

		const QString fbxSelSetName( fbxSelectionSet->GetName() );
		QStringList fbxSelSetNameParts = fbxSelSetName.split( "__" );
		const QString dsFaceGroupName( fbxSelSetNameParts.first() );
		const QString dsMeshNodeName( fbxSelSetNameParts.last() );
		if ( dsMeshNodeName == dsFaceGroupName
			|| dsMeshNodeName != dsMeshNode->getName() )
		{
			continue;
		}

		FbxArray<FbxSelectionNode*> fbxSelectionNodeList;
		FbxArray<FbxObject*> fbxDirectObjectList;
		fbxSelectionSet->GetSelectionNodesAndDirectObjects( fbxSelectionNodeList, fbxDirectObjectList );

		// directly connected objects
		for ( int j = 0, m = fbxDirectObjectList.GetCount(); j < m; j++ )
		{
			const FbxObject* fbxObject = fbxDirectObjectList[j];
			const QString str( fbxObject->GetName() );
			Q_UNUSED( str );
		}

		// selection nodes
		for ( int j = 0, m = fbxSelectionNodeList.GetCount(); j < m; j++ )
		{
			FbxSelectionNode* fbxSelectionNode = fbxSelectionNodeList[j];
			if ( !fbxSelectionNode )
			{
				continue;
			}

			FbxArray<int> fbxFacetIndices;
			fbxSelectionSet->GetFaceSelection( fbxSelectionNode, fbxFacetIndices );

			if ( fbxFacetIndices.GetCount() < 1 )
			{
				continue;
			}

			if ( asFaceGroups )
			{
				const bool created = dsMesh->createFaceGroup( dsFaceGroupName );
				Q_UNUSED( created )

				const int numFacetIndices = fbxFacetIndices.GetCount();

				// create a temporary list of facet indices to use for selection
				DzTSharedPointer<DzFaceGroup> dsFaceGroup( new DzFaceGroup( dsFaceGroupName ) );
				dsFaceGroup->preSizeArray( numFacetIndices );
				for ( int k = 0; k < numFacetIndices; k++ )
				{
					dsFaceGroup->addIndex( fbxFacetIndices.GetAt( k ) );
				}

				// use facet selection state to create face groups;
				// doing it this way more easily handles exclusivity
#if DZ_SDK_4_12_OR_GREATER
				dsMesh->beginFacetSelectionEdit();
#else
				// DzFacetMesh::beginFacetSelectionEdit() is not in the 4.5 SDK,
				// so we attempt to use the meta-object to call the method - since 4.6.3.39

				bool im = QMetaObject::invokeMethod( dsMesh, "beginFacetSelectionEdit" );
				assert( im );
#endif

				dsMesh->deselectAllFacets();

#if DZ_SDK_4_12_OR_GREATER
				dsMesh->selectFacetsByIndexList( dsFaceGroup, true );
#else
				// DzFacetMesh::selectFacetsByIndexList() is not in the 4.5 SDK,
				// so we attempt to use the meta-object to call the method - since 4.6.3.39.
				// If that fails we fall back to doing it ourselves

				if ( !QMetaObject::invokeMethod( dsMesh, "selectFacetsByIndexList",
						Q_ARG( const DzIndexList*, dsFaceGroup ), Q_ARG( bool, true) ) )
				{
					selectFacetsByIndexList( dsMesh, dsFaceGroup );
				}
#endif

				dsMesh->addSelectedFacetsToGroup( dsFaceGroupName );
				dsMesh->deselectAllFacets();

#if DZ_SDK_4_12_OR_GREATER
				dsMesh->finishFacetSelectionEdit();
#else
				// DzFacetMesh::beginFacetSelectionEdit() is not in the 4.5 SDK,
				// so we attempt to use the meta-object to call the method - since 4.6.3.39

				im = QMetaObject::invokeMethod( dsMesh, "finishFacetSelectionEdit" );
				assert( im );
#endif
			}
			else // as a selection group
			{
#if DZ_SDK_4_12_OR_GREATER
				DzSelectionGroup* dsSelectionGrp = dsShape->findFacetSelectionGroup( dsFaceGroupName, true );
				for ( int k = 0; k < fbxFacetIndices.GetCount(); k++ )
				{
					dsSelectionGrp->addIndex( fbxFacetIndices.GetAt( k ) );
				}
#else
				// DzSelectionGroup is not in the 4.5 SDK, but its superclass
				// DzIndexList is and it provides DzIndexList::addIndex()

				// DzFacetShape::findFacetSelectionGroup() is not in the 4.5 SDK,
				// so we attempt to use the meta-object to call the method - since 4.6.3.39

				DzIndexList* dsSelectionGrp = NULL;
				if ( QMetaObject::invokeMethod( dsShape, "findFacetSelectionGroup",
						Q_RETURN_ARG( DzIndexList*, dsSelectionGrp ),
						Q_ARG( const QString&, dsFaceGroupName ), Q_ARG( bool, true ) ) )
				{
					for ( int k = 0, p = fbxFacetIndices.GetCount(); k < p; k++ )
					{
						dsSelectionGrp->addIndex( fbxFacetIndices.GetAt( k ) );
					}
				}
#endif
			}
		}
	}

	// clean up empty face groups
	for ( int i = dsMesh->getNumFaceGroups() - 1; i >= 0; --i )
	{
		DzFaceGroup* dsFaceGroup = dsMesh->getFaceGroup( i );
		if ( dsFaceGroup->count() > 0 )
		{
			continue;
		}

#if DZ_SDK_4_12_OR_GREATER
		const bool removed = dsMesh->removeFaceGroup( dsFaceGroup->getName() );
		Q_UNUSED( removed )
#else
		bool removed = false;
		// DzFacetMesh::removeFaceGroup() is not in the 4.5 SDK, so we attempt
		// to use the meta-object to call the method. If this fails, we attempt to
		// use the deprecated method.

		if ( !QMetaObject::invokeMethod( dsMesh, "removeFaceGroup",
			Q_RETURN_ARG( bool, removed ), Q_ARG( QString, dsFaceGroup->getName() )) )
		{
			removed = dsMesh->removeFacetGroup( dsFaceGroup->getName() );
		}
		Q_UNUSED( removed )
#endif
	}
}

/**
**/
void DzFbxImporter::updateSelectionMap( Node* node )
{
	const QString fbxSelSetName = m_nodeFaceGroupMap.value( node );
	if ( fbxSelSetName.isEmpty() )
	{
		return;
	}

	QStringList fbxSelSetNameParts = fbxSelSetName.split( "__" );
	const QString dsFaceGroupName( fbxSelSetNameParts.first() );
	const QString dsMeshNodeName( fbxSelSetNameParts.last() );

	DzBone* dsBone = qobject_cast<DzBone*>( node->dsNode );
	if ( !dsBone )
	{
		return;
	}

	DzSkeleton* dsSkeleton = qobject_cast<DzSkeleton*>( dsBone->getSkeleton() );
	if ( !dsSkeleton )
	{
		return;
	}

	if ( dsSkeleton->getName() != dsMeshNodeName )
	{
		return;
	}

	const DzObject* dsObject = dsSkeleton->getObject();
	if ( !dsObject )
	{
		return;
	}

	const DzShape* dsShape = dsObject->getCurrentShape();
	if ( !dsShape )
	{
		return;
	}

	const DzFacetMesh* dsMesh = qobject_cast<DzFacetMesh*>( dsShape->getGeometry() );
	if ( !dsMesh )
	{
		return;
	}

	if ( !dsMesh->findFaceGroup( dsFaceGroupName ) )
	{
		return;
	}

	DzSelectionMap* dsSelectionMap = dsSkeleton->getSelectionMap();
	if ( !dsSelectionMap )
	{
		dsSelectionMap = new DzSelectionMap();
		dsSkeleton->setSelectionMap( dsSelectionMap );
	}

	dsSelectionMap->addPair( dsFaceGroupName, dsBone );

	dsSkeleton->setDrawGLBones( false );
}

/**
**/
void DzFbxImporter::fbxImportFaces( FbxMesh* fbxMesh, DzFacetMesh* dsMesh, bool matsAllSame, QMap<QPair<int, int>, int> &edgeMap )
{
	int numEdges = 0;

	const int numPolygons = fbxMesh->GetPolygonCount();

	const FbxGeometryElementPolygonGroup* fbxPolygonGroup = m_includePolygonGroups ?
		fbxMesh->GetElementPolygonGroup( 0 ) : NULL;

	// check whether we have compatible polygon group info;
	// count is 0 since FBX SDK 2020.0;
	// count is as expected with FBX SDK 2019.5 and prior
	const bool compatPolyGroup = fbxPolygonGroup && numPolygons == fbxPolygonGroup->GetIndexArray().GetCount();

	int curGroupIdx = -1;
	for ( int polyIdx = 0; polyIdx < numPolygons; polyIdx++ )
	{
		// active material group
		if ( !matsAllSame )
		{
			for ( int matElemIdx = 0, numMatElements = fbxMesh->GetElementMaterialCount();
				matElemIdx < numMatElements; matElemIdx++ )
			{
				const FbxGeometryElementMaterial* fbxMaterial = fbxMesh->GetElementMaterial( matElemIdx );
				const int polyMatIdx = fbxMaterial->GetIndexArray().GetAt( polyIdx );
				if ( polyMatIdx >= 0 )
				{
					dsMesh->activateMaterial( polyMatIdx );
					break;
				}
			}
		}

		// active face group
		if ( compatPolyGroup )
		{
			const int groupIdx = fbxPolygonGroup->GetIndexArray().GetAt( polyIdx );
			if ( groupIdx != curGroupIdx )
			{
				curGroupIdx = groupIdx;
				dsMesh->activateFaceGroup( "fbx_polygonGroup_" % QString::number( groupIdx ) );
			}
		}

		DzFacet face;

		// facet vertices
		int triFanRoot = -1;
		for ( int polyVertIdx = 0, numPolyVerts = fbxMesh->GetPolygonSize( polyIdx );
			polyVertIdx < numPolyVerts; polyVertIdx++ )
		{
			// quads, tris, lines
			if ( numPolyVerts <= 4 )
			{
				face.m_vertIdx[polyVertIdx] = fbxMesh->GetPolygonVertex( polyIdx, polyVertIdx );
				face.m_normIdx[polyVertIdx] = face.m_vertIdx[polyVertIdx];

				// facet UVs
				for ( int uvElemIdx = 0, numUvElems = fbxMesh->GetElementUVCount();
					uvElemIdx < numUvElems; uvElemIdx++ )
				{
					const FbxGeometryElementUV* fbxGeomUv = fbxMesh->GetElementUV( uvElemIdx );
					switch ( fbxGeomUv->GetMappingMode() )
					{
					case FbxGeometryElement::eByControlPoint:
						switch ( fbxGeomUv->GetReferenceMode() )
						{
						case FbxGeometryElement::eDirect:
							face.m_uvwIdx[polyVertIdx] = face.m_vertIdx[polyVertIdx];
							break;
						case FbxGeometryElement::eIndexToDirect:
							face.m_uvwIdx[polyVertIdx] = fbxGeomUv->GetIndexArray().GetAt( face.m_vertIdx[polyVertIdx] );
							break;
						default:
							break;
						}
						break;
					case FbxGeometryElement::eByPolygonVertex:
						face.m_uvwIdx[polyVertIdx] = fbxMesh->GetTextureUVIndex( polyIdx, polyVertIdx );
						break;
					default:
						break;
					}

					// only do the first UV set
					break;
				}

				if ( polyVertIdx == numPolyVerts - 1 )
				{
					dsMesh->addFacet( face.m_vertIdx, face.m_uvwIdx );
				}
			}
			// n-gons
			else if ( polyVertIdx >= 2 )
			{
				const bool isRoot = polyVertIdx == 2;

				face.m_vertIdx[0] = fbxMesh->GetPolygonVertex( polyIdx, 0 );
				face.m_vertIdx[1] = fbxMesh->GetPolygonVertex( polyIdx, polyVertIdx - 1 );
				face.m_vertIdx[2] = fbxMesh->GetPolygonVertex( polyIdx, polyVertIdx );
				face.m_vertIdx[3] = -1;
				face.m_normIdx[0] = face.m_vertIdx[0];
				face.m_normIdx[1] = face.m_vertIdx[1];
				face.m_normIdx[2] = face.m_vertIdx[2];
				face.m_normIdx[3] = face.m_vertIdx[3];

				if ( isRoot )
				{
					triFanRoot = dsMesh->getNumFacets();
				}

#if DZ_SDK_4_12_OR_GREATER
				face.setTriFanRoot( triFanRoot );
#else
				// DzFacet::setTriFanRoot() is not in the 4.5 SDK, and DzFacet
				// is not derived from QObject, so we must modify the member
				// directly.

				face.m_vertIdx[3] = -(triFanRoot + 2);
#endif

				if ( isRoot )
				{
#if DZ_SDK_4_12_OR_GREATER
					face.setTriFanCount( numPolyVerts - 2 );
#else
					// DzFacet::setTriFanCount() is not in the 4.5 SDK, and DzFacet
					// is not derived from QObject, so we must modify the member
					// directly.

					face.m_edges[3] = -numPolyVerts;
#endif
				}
				else
				{
#if DZ_SDK_4_12_OR_GREATER
					face.clearTriFanCount();
#else
					// DzFacet::clearTriFanCount() is not in the 4.5 SDK, and DzFacet
					// is not derived from QObject, so we must modify the member
					// directly.

					face.m_edges[3] = -1;
#endif
				}

				// facet UVs
				for ( int uvElemIdx = 0, numUvElems = fbxMesh->GetElementUVCount();
					uvElemIdx < numUvElems; uvElemIdx++ )
				{
					const FbxGeometryElementUV* fbxGeomUv = fbxMesh->GetElementUV( uvElemIdx );
					switch ( fbxGeomUv->GetMappingMode() )
					{
					case FbxGeometryElement::eByControlPoint:
						switch ( fbxGeomUv->GetReferenceMode() )
						{
						case FbxGeometryElement::eDirect:
							face.m_uvwIdx[0] = face.m_vertIdx[0];
							face.m_uvwIdx[1] = face.m_vertIdx[polyVertIdx - 1];
							face.m_uvwIdx[2] = face.m_vertIdx[polyVertIdx];
							face.m_uvwIdx[3] = -1;
							break;
						case FbxGeometryElement::eIndexToDirect:
							face.m_uvwIdx[0] = fbxGeomUv->GetIndexArray().GetAt( face.m_vertIdx[0] );
							face.m_uvwIdx[1] = fbxGeomUv->GetIndexArray().GetAt( face.m_vertIdx[polyVertIdx - 1] );
							face.m_uvwIdx[2] = fbxGeomUv->GetIndexArray().GetAt( face.m_vertIdx[polyVertIdx] );
							face.m_uvwIdx[3] = -1;
							break;
						default:
							break;
						}
						break;
					case FbxGeometryElement::eByPolygonVertex:
						face.m_uvwIdx[0] = fbxMesh->GetTextureUVIndex( polyIdx, polyVertIdx );
						face.m_uvwIdx[1] = fbxMesh->GetTextureUVIndex( polyIdx, polyVertIdx - 1 );
						face.m_uvwIdx[2] = fbxMesh->GetTextureUVIndex( polyIdx, polyVertIdx );
						face.m_uvwIdx[3] = -1;
						break;
					default:
						break;
					}

					// only do the first UV set
					break;
				}

#if DZ_SDK_4_12_OR_GREATER
				dsMesh->addFacet( face );
#else
				// DzFacetMesh::addFacet() is not in the 4.5 SDK, so we attempt
				// to use the meta-object to call the method.

				bool im = QMetaObject::invokeMethod( dsMesh, "addFacet",
					Q_ARG( const DzFacet &, face ) );
				assert( im );
#endif

				if ( isRoot )
				{
#if DZ_SDK_4_12_OR_GREATER
					dsMesh->incrementNgons();
#else
					// DzFacetMesh::incrementNgons() is not in the 4.5 SDK, so
					// we attempt to use the meta-object to call the method.

					bool im = QMetaObject::invokeMethod( dsMesh, "incrementNgons" );
					assert( im );
#endif
				}
			}


			{
				const int polyVertNextIdx = (polyVertIdx + 1) % numPolyVerts;

				const int edgeVertA = fbxMesh->GetPolygonVertex( polyIdx, polyVertIdx );
				const int edgeVertB = fbxMesh->GetPolygonVertex( polyIdx, polyVertNextIdx );
				QPair<int, int> edgeVertPair( qMin( edgeVertA, edgeVertB ), qMax( edgeVertA, edgeVertB ) );
				if ( !edgeMap.contains( edgeVertPair ) )
				{
					edgeMap[edgeVertPair] = numEdges;
					numEdges++;
				}
			}
		}
	}
}

/**
**/
void DzFbxImporter::fbxImportSubdEdgeWeights( FbxMesh* fbxMesh, DzFacetMesh* dsMesh, QMap<QPair<int, int>, int> edgeMap, bool &enableSubd )
{
	for ( int i = 0, n = fbxMesh->GetElementEdgeCreaseCount(); i < n; i++ )
	{
		const FbxGeometryElementCrease* fbxSubdEdgeCrease = fbxMesh->GetElementEdgeCrease( i );
		const int numCreases = fbxSubdEdgeCrease->GetDirectArray().GetCount();
		Q_UNUSED( numCreases )

		QMap< QPair<int, int>, int >::iterator edgeMapIt;
		for ( edgeMapIt = edgeMap.begin(); edgeMapIt != edgeMap.end(); ++edgeMapIt )
		{
			const int edgeIdx = edgeMapIt.value();
			const float weight = fbxSubdEdgeCrease->GetDirectArray().GetAt( edgeIdx );
			const int edgeVertA = edgeMapIt.key().first;
			const int edgeVertB = edgeMapIt.key().second;
			if ( weight > 0 )
			{
				enableSubd = true;
#if DZ_SDK_4_12_OR_GREATER
				dsMesh->setEdgeWeight( edgeVertA, edgeVertB, weight );
#else
				// DzFacetMesh::setEdgeWeight() is not in the 4.5 SDK, so we
				// attempt to use the meta-object to call the method.

				bool im = QMetaObject::invokeMethod( dsMesh, "setEdgeWeight",
					Q_ARG( int, edgeVertA ), Q_ARG( int, edgeVertB ), Q_ARG( int, weight ) );
				assert( im );
#endif
			}
		}

		// only do the first
		break;
	}
}

/**
**/
DzWeightMapPtr DzFbxImporter::fbxImportSkinningBlendWeights( int numVertices, const FbxSkin* fbxSkin )
{
	if ( numVertices < 1
		|| !fbxSkin
		|| fbxSkin->GetSkinningType() != FbxSkin::eBlend )
	{
		return NULL;
	}

	const int numBlendIndices = fbxSkin->GetControlPointIndicesCount();
	const int* fbxBlendIndices = fbxSkin->GetControlPointIndices();
	if ( numBlendIndices <= 0 || !fbxBlendIndices )
	{
		return NULL;
	}

	DzWeightMapPtr dsBlendWeights = new DzWeightMap( numVertices, "Blend Weights" );
	unsigned short* dsWeightValues = dsBlendWeights->getWeights();
	for ( int bwIdx = 0; bwIdx < numBlendIndices; ++bwIdx )
	{
		const int idx = fbxBlendIndices[bwIdx];
		if ( idx > numVertices )
		{
			continue;
		}

		const double fbxBlendWeight = fbxSkin->GetControlPointBlendWeights()[bwIdx];
		dsWeightValues[idx] = DZ_USHORT_MAX * fbxBlendWeight;
	}

#if DZ_SDK_4_12_OR_GREATER
	dsBlendWeights->reduceWeightMemory();
#endif

	return dsBlendWeights;
}

/**
**/
void DzFbxImporter::fbxImportMorph( FbxBlendShape* fbxBlendShape, DzObject* dsObject, int numVertices, FbxVector4* fbxVertices )
{
	if ( !fbxBlendShape
		|| !dsObject
		|| numVertices < 1 )
	{
		return;
	}

	DzPnt3* values = new DzPnt3[numVertices];

	const int numBlendShapeChannels = fbxBlendShape->GetBlendShapeChannelCount();

	DzProgress progress( "Morphs", numBlendShapeChannels );
	for ( int blendShapeChanIdx = 0; blendShapeChanIdx < numBlendShapeChannels; blendShapeChanIdx++ )
	{
		FbxBlendShapeChannel* fbxBlendChannel = fbxBlendShape->GetBlendShapeChannel( blendShapeChanIdx );

		DzMorph* dsMorph = new DzMorph;
		dsMorph->setName( fbxBlendChannel->GetName() );
		DzMorphDeltas* dsDeltas = dsMorph->getDeltas();

		DzFloatProperty* morphControl = NULL;
#if DZ_SDK_4_12_OR_GREATER
		morphControl = dsMorph->getValueControl();
#else
		// DzMorph::getValueControl() is not in the 4.5 SDK, but DzMorph::getValueChannel()
		// was, so we use the previous name.

		morphControl = dsMorph->getValueChannel();
#endif

		applyFbxCurve( fbxBlendChannel->DeformPercent.GetCurve( m_fbxAnimLayer ), morphControl, 0.01 );

		for ( int vertIdx = 0; vertIdx < numVertices; vertIdx++ )
		{
			values[vertIdx][0] = 0;
			values[vertIdx][1] = 0;
			values[vertIdx][2] = 0;
		}

		for ( int tgtShapeIdx = 0, numTgtShapes = fbxBlendChannel->GetTargetShapeCount();
			tgtShapeIdx < numTgtShapes; tgtShapeIdx++ )
		{
			FbxShape* fbxTargetShape = fbxBlendChannel->GetTargetShape( tgtShapeIdx );
			FbxVector4* fbxTargetShapeVerts = fbxTargetShape->GetControlPoints();
			int* fbxTgtShapeVertIndices = fbxTargetShape->GetControlPointIndices();
			//double weight = fbxBlendChannel->GetTargetShapeFullWeights()[k];

			if ( fbxTgtShapeVertIndices )
			{
				for ( int tgtShapeVertIndicesIdx = 0, numTgtShapeVertIndices = fbxTargetShape->GetControlPointIndicesCount();
					tgtShapeVertIndicesIdx < numTgtShapeVertIndices; tgtShapeVertIndicesIdx++ )
				{
					const int vertIdx = fbxTgtShapeVertIndices[tgtShapeVertIndicesIdx];
					values[vertIdx][0] = fbxTargetShapeVerts[vertIdx][0] - fbxVertices[vertIdx][0];
					values[vertIdx][1] = fbxTargetShapeVerts[vertIdx][1] - fbxVertices[vertIdx][1];
					values[vertIdx][2] = fbxTargetShapeVerts[vertIdx][2] - fbxVertices[vertIdx][2];
				}
			}
			else
			{
				for ( int vertIdx = 0, numTgtShapeVerts = fbxTargetShape->GetControlPointsCount();
					vertIdx < numTgtShapeVerts; vertIdx++ )
				{
					values[vertIdx][0] = fbxTargetShapeVerts[vertIdx][0] - fbxVertices[vertIdx][0];
					values[vertIdx][1] = fbxTargetShapeVerts[vertIdx][1] - fbxVertices[vertIdx][1];
					values[vertIdx][2] = fbxTargetShapeVerts[vertIdx][2] - fbxVertices[vertIdx][2];
				}
			}
		}

		DzIntArray indexes;
		DzTArray<DzVec3> deltas;
		for ( int vertIdx = 0; vertIdx < numVertices; vertIdx++ )
		{
			if ( values[vertIdx][0] != 0 || values[vertIdx][1] != 0 || values[vertIdx][2] != 0 )
			{
				indexes.append( vertIdx );
				deltas.append( DzVec3( values[vertIdx][0], values[vertIdx][1], values[vertIdx][2] ) );
			}
		}
		dsDeltas->addDeltas( indexes, deltas, false );
		dsObject->addModifier( dsMorph );

		progress.step();
	}

	delete[] values;
}

/**
**/
void DzFbxImporter::fbxImportMeshModifiers( Node* node, FbxMesh* fbxMesh, DzObject* dsObject, DzFigure* dsFigure, int numVertices, FbxVector4* fbxVertices )
{
	for ( int deformerIdx = 0, numDeformers = fbxMesh->GetDeformerCount(); deformerIdx < numDeformers; deformerIdx++ )
	{
		FbxDeformer* fbxDeformer = fbxMesh->GetDeformer( deformerIdx );

		// skin binding
		if ( FbxSkin* fbxSkin = FbxCast<FbxSkin>( fbxDeformer ) )
		{
			if ( !dsFigure )
			{
				continue;
			}

			Skinning skinning;
			skinning.node = node;
			skinning.fbxSkin = fbxSkin;
			skinning.dsFigure = dsFigure;
			skinning.numVertices = numVertices;
			skinning.blendWeights = fbxImportSkinningBlendWeights( numVertices, fbxSkin );
			m_skins.push_back( skinning );
		}
		// morphs
		else if ( FbxBlendShape* fbxBlendShape = FbxCast<FbxBlendShape>( fbxDeformer ) )
		{
			fbxImportMorph( fbxBlendShape, dsObject, numVertices, fbxVertices );
		}
	}
}

/**
**/
void DzFbxImporter::fbxImportMesh( Node* node, FbxNode* fbxNode, DzNode* dsMeshNode )
{
	FbxMesh* fbxMesh = fbxNode->GetMesh();

	const QString dsName = dsMeshNode ? dsMeshNode->getName() : fbxNode->GetName();

	DzObject* dsObject = new DzObject();
	dsObject->setName( !dsName.isEmpty() ? dsName : "object" );

	DzFacetMesh* dsMesh = new DzFacetMesh();
	dsMesh->setName( !dsName.isEmpty() ? dsName : "geometry" );

#if DZ_SDK_4_12_OR_GREATER
	DzFacetShape* dsShape = new DzGraftingFigureShape();
#else
	// DzGraftingFigureShape is not in the 4.5 SDK, but if the version of the
	// application has the factory for the class we can use it to create an
	// instance or fallback to the base class that is in the 4.5 SDK

	DzFacetShape* dsShape;
	if ( const DzClassFactory* factory = dzApp->findClassFactory( "DzGraftingFigureShape" ) )
	{
		dsShape = qobject_cast<DzFacetShape*>( factory->createInstance() );
	}
	else
	{
		dsShape = new DzFacetShape();
	}
#endif
	dsShape->setName( !dsName.isEmpty() ? dsName : "shape" );

	DzFigure* dsFigure = qobject_cast<DzFigure*>( dsMeshNode );

	DzVec3 offset( 0, 0, 0 );
	if ( dsFigure )
	{
		offset = dsFigure->getOrigin();
	}
	else if (m_compatibilityMode)
	{
		FbxNode* fbxMeshNode = fbxMesh->GetNode();
		if (fbxMeshNode)
		{
			FbxVector4 translation = fbxMeshNode->EvaluateGlobalTransform().GetT();
			for (int i = 0; i < 3; i++)
			{
				offset[i] = translation[i];
			}
		}
	}

	// begin the edit
	dsMesh->beginEdit();

	const int numVertices = fbxMesh->GetControlPointsCount();
	FbxVector4* fbxVertices = fbxMesh->GetControlPoints();

	// DB 4-27-2023: evaluate bindpose onto vertexbuffer
	//   this should fix multiple bindposes as well as rootnode offsets and rotations
	fbxsdk::FbxArray<int> indexArray;
	fbxsdk::PoseList poseList;
	FbxNode* skeletonRoot = FbxTools::findAssociatedSkeletonRoot(fbxMesh);
	if (FbxPose::GetBindPoseContaining(fbxMesh->GetScene(), skeletonRoot, poseList, indexArray) == true)
	{
		int i = indexArray[0];
		FbxPose* pBindPose = poseList[0];
		if (pBindPose->IsBindPose())
		{
			FbxAMatrix correctionMatrix;
			if (FbxTools::getBindMatrixFromCluster(fbxNode, correctionMatrix) == false)
			{
				correctionMatrix.SetIdentity();
			}
			FbxTools::bakePoseToVertexBuffer(fbxVertices, &correctionMatrix, pBindPose, fbxMesh);
			FbxTools::bakePoseToBindMatrix(fbxMesh, pBindPose);
		}
	}

	fbxImportVertices( numVertices, fbxVertices, dsMesh, offset );

	fbxImportUVs( fbxMesh, dsMesh );

	bool enableSubd = false;
	fbxImportSubdVertexWeights( fbxMesh, dsMesh, enableSubd );

	bool matsAllSame;
	fbxImportMaterials( fbxNode, fbxMesh, dsMesh, dsShape, matsAllSame );

	QMap< QPair< int, int >, int > edgeMap;
	fbxImportFaces( fbxMesh, dsMesh, matsAllSame, edgeMap );

	fbxImportSubdEdgeWeights( fbxMesh, dsMesh, edgeMap, enableSubd );

	// end the edit
	dsMesh->finishEdit();

	dsShape->setFacetMesh( dsMesh );

	setSubdEnabled( enableSubd, dsMesh, dsShape );

	dsObject->addShape( dsShape );
	dsMeshNode->setObject( dsObject );

	fbxImportPolygonSets( dsMeshNode, dsMesh, dsShape );

	fbxImportMeshModifiers( node, fbxMesh, dsObject, dsFigure, numVertices, fbxVertices );
}

/**
**/
void DzFbxImporter::setSubdEnabled( bool onOff, DzFacetMesh* dsMesh, DzFacetShape* dsShape )
{
	if ( !onOff )
	{
		return;
	}

	dsMesh->enableSubDivision( true );

	if ( DzEnumProperty* lodControl = dsShape->getLODControl() )
	{
		lodControl->setValue( lodControl->getNumItems() - 1 ); //set to high res
		lodControl->setDefaultValue( lodControl->getNumItems() - 1 ); //set to high res
	}
}

/**
**/
void DzFbxImporter::applyFbxCurve( FbxAnimCurve* fbxCurve, DzFloatProperty* dsProperty, double scale )
{
	if ( !fbxCurve || !dsProperty )
	{
		return;
	}

	dsProperty->deleteAllKeys();

	for ( int i = 0; i < fbxCurve->KeyGetCount(); i++ )
	{
		const double fbxTime = fbxCurve->KeyGetTime( i ).GetSecondDouble();
		const double fbxValue = fbxCurve->KeyGetValue( i );
		DzTime dsTime = static_cast< DzTime >((fbxTime * DZ_TICKS_PER_SECOND) + 0.5f); //round to nearest tick
		m_dsEndTime = qMax( m_dsEndTime, dsTime );

		dsProperty->setValue( dsTime, fbxValue * scale );
	}
}

///////////////////////////////////////////////////////////////////////
// DzFbxImportFrame
///////////////////////////////////////////////////////////////////////

struct DzFbxImportFrame::Data
{
	Data( DzFbxImporter* importer ) :
		m_importer( importer ),
		m_includeRotationLimitsCbx( NULL ),
		m_includeAnimationCbx( NULL ),
		m_animationTakeCmb( NULL ),
		m_includePolygonSetsCbx( NULL ),
		m_includePolygonGroupsCbx( NULL ),
		m_studioNodeNameLabelCbx( NULL ),
		m_studioPresentationCbx( NULL ),
		m_studioSelectionMapCbx( NULL ),
		m_studioSceneIDsCbx( NULL )
	{}

	DzFbxImporter*	m_importer;

	QCheckBox*		m_includeRotationLimitsCbx;
	QCheckBox*		m_includeAnimationCbx;
	QComboBox*		m_animationTakeCmb;

	QCheckBox*		m_includePolygonSetsCbx;
	QCheckBox*		m_includePolygonGroupsCbx;

	QCheckBox*		m_studioNodeNameLabelCbx;
	QCheckBox*		m_studioPresentationCbx;
	QCheckBox*		m_studioSelectionMapCbx;
	QCheckBox*		m_studioSceneIDsCbx;
};

namespace
{

QGroupBox* createCollapsibleGroupBox( const QString &title, const QString &basename, bool collapsed = false )
{
#if DZ_SDK_4_12_OR_GREATER
	DzCollapsibleGroupBox* groupBox = new DzCollapsibleGroupBox( title );
	groupBox->setObjectName( basename % "GBox" );
	groupBox->setCollapsed( collapsed );
#else
	QGroupBox* groupBox = NULL;
	if ( const DzClassFactory* factory = dzApp->findClassFactory( "DzCollapsibleGroupBox" ) )
	{
		groupBox = qobject_cast<QGroupBox*>( factory->createInstance() );
	}

	if ( !groupBox )
	{
		groupBox = new QGroupBox();
	}

	groupBox->setObjectName( basename % "GBox" );
	groupBox->setTitle( title );
#endif //DZ_SDK_4_12_OR_GREATER

	return groupBox;
}

QScrollArea* createScrollableWidget( QWidget* widgetToScroll, int margin, int spacing, const QString &nameBase )
{
	QWidget* scrollableWgt = new QWidget();
	scrollableWgt->setObjectName( nameBase % "ScrollWgt" );

	QVBoxLayout* scrollableLyt = new QVBoxLayout();
	scrollableLyt->setMargin( margin );
	scrollableLyt->setSpacing( spacing );

	scrollableLyt->addWidget( widgetToScroll );

	scrollableLyt->addStretch();

	scrollableWgt->setLayout( scrollableLyt );

	QScrollArea* scrollArea = new QScrollArea();
	scrollArea->setObjectName( nameBase % "ScrollArea" );
	scrollArea->setWidgetResizable( true );
	scrollArea->setWidget( scrollableWgt );

	return scrollArea;
}

const char* c_none = QT_TRANSLATE_NOOP( "DzFbxImportFrame", "<None>" );

} //namespace

/**
**/
DzFbxImportFrame::DzFbxImportFrame( DzFbxImporter* importer ) :
	DzFileIOFrame( tr( "FBX Import Options" ) ), m_data( new Data( importer ) )
{
	const QString name( "FbxImport" );

	const int margin = style()->pixelMetric( DZ_PM_GeneralMargin );
	const int btnHeight = style()->pixelMetric( DZ_PM_ButtonHeight );

	QVector<QLabel*> leftLabels;
	leftLabels.reserve( 10 );

	QVBoxLayout* mainLyt = new QVBoxLayout();
	mainLyt->setSpacing( margin );
	mainLyt->setMargin( margin );

	// Format
	QGroupBox* formatGBox = new QGroupBox( tr( "Format :" ) );
	formatGBox->setObjectName( name % "FormatGBox" );

	QGridLayout* formatLyt = new QGridLayout();
	formatLyt->setSpacing( margin );
	formatLyt->setMargin( margin );
	formatLyt->setColumnStretch( 1, 1 );

	int row = 0;

	QLabel* lbl = new QLabel( tr( "Version:" ) );
	lbl->setObjectName( name % "FileVersionLbl" );
	lbl->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
	formatLyt->addWidget( lbl, row, 0 );
	leftLabels.push_back( lbl );

	lbl = new QLabel( importer->getFileVersion() );
	lbl->setObjectName( name % "FileVersionValueLbl" );
	lbl->setTextInteractionFlags( Qt::TextBrowserInteraction );
	formatLyt->addWidget( lbl, row++, 1 );

	lbl = new QLabel( tr( "Creator:" ) );
	lbl->setObjectName( name % "FileCreatorLbl" );
	lbl->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
	formatLyt->addWidget( lbl, row, 0 );
	leftLabels.push_back( lbl );

	lbl = new QLabel( importer->getFileCreator() );
	lbl->setObjectName( name % "FileCreatorValueLbl" );
	lbl->setTextInteractionFlags( Qt::TextBrowserInteraction );
	formatLyt->addWidget( lbl, row++, 1 );

	formatGBox->setLayout( formatLyt );

	mainLyt->addWidget( formatGBox );


	// Scene Info
	QGroupBox* sceneInfoGBox = createCollapsibleGroupBox( tr( "Scene :" ), name % "SceneInfo", true );

	QGridLayout* sceneInfoLyt = new QGridLayout();
	sceneInfoLyt->setSpacing( margin );
	sceneInfoLyt->setMargin( margin );
	sceneInfoLyt->setColumnStretch( 1, 1 );

	row = 0;

	const QString sceneAuthor = importer->getSceneAuthor();
	if ( !sceneAuthor.isEmpty() )
	{
		lbl = new QLabel( tr( "Author:" ) );
		lbl->setObjectName( name % "SceneAuthorLbl" );
		lbl->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
		sceneInfoLyt->addWidget( lbl, row, 0 );
		leftLabels.push_back( lbl );

		lbl = new QLabel( sceneAuthor );
		lbl->setObjectName( name % "SceneAuthorValueLbl" );
		lbl->setTextInteractionFlags( Qt::TextBrowserInteraction );
		sceneInfoLyt->addWidget( lbl, row++, 1 );
	}

	const QString sceneTitle = importer->getSceneTitle();
	if ( !sceneTitle.isEmpty() )
	{
		lbl = new QLabel( tr( "Title:" ) );
		lbl->setObjectName( name % "SceneTitleLbl" );
		lbl->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
		sceneInfoLyt->addWidget( lbl, row, 0 );
		leftLabels.push_back( lbl );

		lbl = new QLabel( sceneTitle );
		lbl->setObjectName( name % "SceneTitleValueLbl" );
		lbl->setTextInteractionFlags( Qt::TextBrowserInteraction );
		sceneInfoLyt->addWidget( lbl, row++, 1 );
	}

	const QString sceneSubject = importer->getSceneSubject();
	if ( !sceneSubject.isEmpty() )
	{
		lbl = new QLabel( tr( "Subject:" ) );
		lbl->setObjectName( name % "SceneSubjectLbl" );
		lbl->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
		sceneInfoLyt->addWidget( lbl, row, 0 );
		leftLabels.push_back( lbl );

		lbl = new QLabel( sceneSubject );
		lbl->setObjectName( name % "SceneSubjectValueLbl" );
		lbl->setTextInteractionFlags( Qt::TextBrowserInteraction );
		sceneInfoLyt->addWidget( lbl, row++, 1 );
	}

	const QString sceneKeywords = importer->getSceneKeywords();
	if ( !sceneKeywords.isEmpty() )
	{
		lbl = new QLabel( tr( "Keywords:" ) );
		lbl->setObjectName( name % "SceneKeywordsLbl" );
		lbl->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
		sceneInfoLyt->addWidget( lbl, row, 0 );
		leftLabels.push_back( lbl );

		lbl = new QLabel( sceneKeywords );
		lbl->setObjectName( name % "SceneKeywordsValueLbl" );
		lbl->setTextInteractionFlags( Qt::TextBrowserInteraction );
		sceneInfoLyt->addWidget( lbl, row++, 1 );
	}

	const QString sceneRevision = importer->getSceneRevision();
	if ( !sceneRevision.isEmpty() )
	{
		lbl = new QLabel( tr( "Revision:" ) );
		lbl->setObjectName( name % "SceneRevisionLbl" );
		lbl->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
		sceneInfoLyt->addWidget( lbl, row, 0 );
		leftLabels.push_back( lbl );

		lbl = new QLabel( sceneRevision );
		lbl->setObjectName( name % "SceneRevisionValueLbl" );
		lbl->setTextInteractionFlags( Qt::TextBrowserInteraction );
		sceneInfoLyt->addWidget( lbl, row++, 1 );
	}

	const QString sceneComment = importer->getSceneComment();
	if ( !sceneComment.isEmpty() )
	{
		lbl = new QLabel( tr( "Comment:" ) );
		lbl->setObjectName( name % "SceneCommentLbl" );
		lbl->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
		sceneInfoLyt->addWidget( lbl, row, 0 );
		leftLabels.push_back( lbl );

		lbl = new QLabel( sceneComment );
		lbl->setObjectName( name % "SceneCommentValueLbl" );
		lbl->setTextInteractionFlags( Qt::TextBrowserInteraction );
		sceneInfoLyt->addWidget( lbl, row++, 1 );
	}

	const QString vendor = importer->getOriginalAppVendor();
	if ( !vendor.simplified().isEmpty() )
	{
		lbl = new QLabel( tr( "Vendor:" ) );
		lbl->setObjectName( name % "VendorLbl" );
		lbl->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
		sceneInfoLyt->addWidget( lbl, row, 0 );
		leftLabels.push_back( lbl );

		lbl = new QLabel( vendor );
		lbl->setObjectName( name % "VendorValueLbl" );
		lbl->setTextInteractionFlags( Qt::TextBrowserInteraction );
		sceneInfoLyt->addWidget( lbl, row++, 1 );
	}

	const QString application = QString( "%1 %2" )
					.arg( importer->getOriginalAppName() )
					.arg( importer->getOriginalAppVersion() );
	if ( !application.simplified().isEmpty() )
	{
		lbl = new QLabel( tr( "Application:" ) );
		lbl->setObjectName( name % "ApplicationLbl" );
		lbl->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
		sceneInfoLyt->addWidget( lbl, row, 0 );
		leftLabels.push_back( lbl );

		lbl = new QLabel( application );
		lbl->setObjectName( name % "ApplicationValueLbl" );
		lbl->setTextInteractionFlags( Qt::TextBrowserInteraction );
		sceneInfoLyt->addWidget( lbl, row++, 1 );
	}

#if DZ_SDK_4_12_OR_GREATER
	if ( DzCollapsibleGroupBox* sceneInfoCGBox = qobject_cast<DzCollapsibleGroupBox*>( sceneInfoGBox ) )
	{
		sceneInfoCGBox->addLayout( sceneInfoLyt );
	}
	else
	{
		sceneInfoGBox->setLayout( sceneInfoLyt );
	}
#else
	sceneInfoGBox->setLayout( sceneInfoLyt );
#endif //DZ_SDK_4_12_OR_GREATER

	mainLyt->addWidget( sceneInfoGBox );

	QWidget* scrollableOptionsWgt = new QWidget();
	scrollableOptionsWgt->setObjectName( name % "ScrollableOptionsWgt" );


	QVBoxLayout* scrollableOptionsLyt = new QVBoxLayout();
	scrollableOptionsLyt->setSpacing( margin );
	scrollableOptionsLyt->setMargin( 0 );


	// Properties
	QGroupBox* propertiesGBox = new QGroupBox( tr( "Properties :" ) );
	propertiesGBox->setObjectName( name % "PropertiesGBox" );

	QVBoxLayout* propertiesLyt = new QVBoxLayout();
	propertiesLyt->setSpacing( margin );
	propertiesLyt->setMargin( margin );

	m_data->m_includeRotationLimitsCbx = new QCheckBox();
	m_data->m_includeRotationLimitsCbx->setObjectName( name % "IncludeRotationLimitsCbx" );
	m_data->m_includeRotationLimitsCbx->setText( tr( "Include Rotation Limits" ) );
	propertiesLyt->addWidget( m_data->m_includeRotationLimitsCbx );
	DzConnect( m_data->m_includeRotationLimitsCbx, SIGNAL(toggled(bool)),
		importer, SLOT(setRotationLimits(bool)) );

	m_data->m_includeAnimationCbx = new QCheckBox();
	m_data->m_includeAnimationCbx->setObjectName( name % "IncludeAnimationCbx" );
	m_data->m_includeAnimationCbx->setText( tr( "Include Animation" ) );
	propertiesLyt->addWidget( m_data->m_includeAnimationCbx );
	DzConnect( m_data->m_includeAnimationCbx, SIGNAL(toggled(bool)),
		importer, SLOT(setIncludeAnimations(bool)) );

	m_data->m_animationTakeCmb = new QComboBox();
	m_data->m_animationTakeCmb->setObjectName( name % "TakeToImportCmb" );
	m_data->m_animationTakeCmb->addItem( tr( c_none ) );
	//m_data->m_animationTakeCmb->insertSeparator( m_data->m_animationTakeCmb->count() );
	m_data->m_animationTakeCmb->addItems( importer->getAnimStackNames() );
	m_data->m_animationTakeCmb->setCurrentIndex( 0 );
	m_data->m_animationTakeCmb->setFixedHeight( btnHeight );
	m_data->m_animationTakeCmb->setEnabled( false );
	propertiesLyt->addWidget( m_data->m_animationTakeCmb );
	DzConnect( m_data->m_animationTakeCmb, SIGNAL(activated(const QString&)),
		importer, SLOT(setTakeName(const QString&)) );

	DzConnect( m_data->m_includeAnimationCbx, SIGNAL(toggled(bool)),
		m_data->m_animationTakeCmb, SLOT(setEnabled(bool)) );

	propertiesGBox->setLayout( propertiesLyt );

	scrollableOptionsLyt->addWidget( propertiesGBox );


	// Geometry
	QGroupBox* geometryGBox = new QGroupBox( tr( "Geometry :" ) );
	geometryGBox->setObjectName( name % "GeometryGBox" );

	QBoxLayout* geometryLyt = new QVBoxLayout();
	geometryLyt->setSpacing( margin );
	geometryLyt->setMargin( margin );

	m_data->m_includePolygonSetsCbx = new QCheckBox();
	m_data->m_includePolygonSetsCbx->setObjectName( name % "IncludePolygonSetsCbx" );
	m_data->m_includePolygonSetsCbx->setText( tr( "Include Polygon Sets" ) );
	geometryLyt->addWidget( m_data->m_includePolygonSetsCbx );
	DzConnect( m_data->m_includePolygonSetsCbx, SIGNAL(toggled(bool)),
		importer, SLOT(setIncludePolygonSets(bool)) );

	m_data->m_includePolygonGroupsCbx = new QCheckBox();
	m_data->m_includePolygonGroupsCbx->setObjectName( name % "IncludePolygonGroupsCbx" );
	m_data->m_includePolygonGroupsCbx->setText( tr( "Include Polygon Groups" ) );
	geometryLyt->addWidget( m_data->m_includePolygonGroupsCbx );
	DzConnect( m_data->m_includePolygonGroupsCbx, SIGNAL(toggled(bool)),
		importer, SLOT(setIncludePolygonGroups(bool)) );

	geometryGBox->setLayout( geometryLyt );

	scrollableOptionsLyt->addWidget( geometryGBox );


	// Custom Data
	QGroupBox* customDataGBox = new QGroupBox( tr( "Custom Data :" ) );
	customDataGBox->setObjectName( name % "CustomDataGBox" );

	QVBoxLayout* customDataLyt = new QVBoxLayout();
	customDataLyt->setSpacing( margin );
	customDataLyt->setMargin( margin );

	m_data->m_studioNodeNameLabelCbx = new QCheckBox();
	m_data->m_studioNodeNameLabelCbx->setObjectName( name % "IncludeNodeNameLabelCbx" );
	m_data->m_studioNodeNameLabelCbx->setText( tr( "Include Node Names/Labels" ) );
	customDataLyt->addWidget( m_data->m_studioNodeNameLabelCbx );
	DzConnect( m_data->m_studioNodeNameLabelCbx, SIGNAL(toggled(bool)),
		importer, SLOT(setStudioNodeNamesLabels(bool)) );

	m_data->m_studioPresentationCbx = new QCheckBox();
	m_data->m_studioPresentationCbx->setObjectName( name % "IncludePresentationCbx" );
	m_data->m_studioPresentationCbx->setText( tr( "Include Presentation" ) );
	customDataLyt->addWidget( m_data->m_studioPresentationCbx );
	DzConnect( m_data->m_studioPresentationCbx, SIGNAL(toggled(bool)),
		importer, SLOT(setStudioNodePresentation(bool)) );

	m_data->m_studioSelectionMapCbx = new QCheckBox();
	m_data->m_studioSelectionMapCbx->setObjectName( name % "IncludeSelectionMapCbx" );
	m_data->m_studioSelectionMapCbx->setText( tr( "Include Node Selection Map" ) );
	customDataLyt->addWidget( m_data->m_studioSelectionMapCbx );
	DzConnect( m_data->m_studioSelectionMapCbx, SIGNAL(toggled(bool)),
		importer, SLOT(setStudioNodeSelectionMap(bool)) );

	m_data->m_studioSceneIDsCbx = new QCheckBox();
	m_data->m_studioSceneIDsCbx->setObjectName( name % "IncludeSceneIDssCbx" );
	m_data->m_studioSceneIDsCbx->setText( tr( "Include Scene IDs" ) );
	customDataLyt->addWidget( m_data->m_studioSceneIDsCbx );
	DzConnect( m_data->m_studioSceneIDsCbx, SIGNAL(toggled(bool)),
		importer, SLOT(setStudioSceneIDs(bool)) );

	customDataGBox->setLayout( customDataLyt );

	scrollableOptionsLyt->addWidget( customDataGBox );

	scrollableOptionsWgt->setLayout( scrollableOptionsLyt );

	QScrollArea* scrollableOptionsArea = createScrollableWidget( scrollableOptionsWgt, margin, margin, name % "Options" );


	// Footer
	const QString errorList = importer->getErrorList().join( "\n" );

	QGroupBox* reportGrp = new QGroupBox();
	reportGrp->setObjectName( name % "PreImportReportGBox" );
	reportGrp->setTitle( tr( "Pre-Import Report :" ) );

	QVBoxLayout* reportLyt = new QVBoxLayout();
	reportLyt->setSpacing( margin );
	reportLyt->setMargin( margin );

	QLabel* preImportLbl = new QLabel();
	preImportLbl->setObjectName( name % "PreImportReportLbl" );
	preImportLbl->setText( !errorList.isEmpty() ? errorList : tr( "Import Ready." ) );
	preImportLbl->setTextInteractionFlags( Qt::TextBrowserInteraction );

	QScrollArea* preImportScroll = createScrollableWidget( preImportLbl, margin, margin, name % "PreImportReport" );

	reportLyt->addWidget( preImportScroll, 1 );

	reportGrp->setLayout( reportLyt );


	DzDynamicDividerWgt* optionsStagingDvdr = new DzDynamicDividerWgt();
	optionsStagingDvdr->setObjectName( name % "OptionsReportDvdr" );
	optionsStagingDvdr->setDividerBar( Qt::Horizontal, DzDividerBar::BSThin );
	optionsStagingDvdr->setMargin( margin );
	optionsStagingDvdr->setFirstWidget( scrollableOptionsArea );
	optionsStagingDvdr->setSecondWidget( reportGrp );
	optionsStagingDvdr->setPreferredDividerPercent( 0.7 );
	mainLyt->addWidget( optionsStagingDvdr, 10 );

	setLayout( mainLyt );

	// --------

	int leftWidth = 0;
	for ( int i = 0, n = leftLabels.count(); i < n; ++i )
	{
		lbl = leftLabels.at( i );
		const int minWidth = lbl->minimumSizeHint().width();
		if ( minWidth > leftWidth )
		{
			leftWidth = minWidth;
		}
	}

	for ( int i = 0, n = leftLabels.count(); i < n; ++i )
	{
		lbl = leftLabels.at( i );
		lbl->setFixedWidth( leftWidth );
	}

	DzFbxImportFrame::resetOptions();
}

/**
**/
DzFbxImportFrame::~DzFbxImportFrame()
{}

/**
**/
void DzFbxImportFrame::setOptions( const DzFileIOSettings* settings, const QString &filename )
{
	assert( settings );
	if ( !settings )
	{
		return;
	}

	// Properties
	m_data->m_includeRotationLimitsCbx->setChecked( settings->getBoolValue( c_optIncRotationLimits, c_defaultIncludeRotationLimits ) );
	m_data->m_includeAnimationCbx->setChecked( settings->getBoolValue( c_optIncAnimations, c_defaultIncludeAnimations ) );
	const QString take = settings->getStringValue( c_optTake, QString() );
	for ( int i = 0; i < m_data->m_animationTakeCmb->count(); i++ )
	{
		if ( m_data->m_animationTakeCmb->itemText( i ) == take )
		{
			m_data->m_animationTakeCmb->setCurrentIndex( i );
			break;
		}
	}

	// Geometry
	m_data->m_includePolygonSetsCbx->setChecked( settings->getBoolValue( c_optIncPolygonSets, c_defaultIncludePolygonSets ) );
	m_data->m_includePolygonGroupsCbx->setChecked( settings->getBoolValue( c_optIncPolygonGroups, c_defaultIncludePolygonGroups ) );

	// Custom Data
	m_data->m_studioNodeNameLabelCbx->setChecked( settings->getBoolValue( c_optStudioNodeNamesLabels, c_defaultStudioNodeNames ) );
	m_data->m_studioPresentationCbx->setChecked( settings->getBoolValue( c_optStudioPresentation, c_defaultStudioNodePresentation ) );
	m_data->m_studioSelectionMapCbx->setChecked( settings->getBoolValue( c_optStudioNodeSelectionMap, c_defaultStudioNodeSelectionMap ) );
	m_data->m_studioSceneIDsCbx->setChecked( settings->getBoolValue( c_optStudioSceneIDs, c_defaultStudioSceneIDs ) );
}

/**
**/
void DzFbxImportFrame::getOptions( DzFileIOSettings* settings ) const
{
	assert( settings );
	if ( !settings )
	{
		return;
	}

	// Properties
	settings->setBoolValue( c_optIncRotationLimits, m_data->m_includeRotationLimitsCbx->isChecked() );
	settings->setBoolValue( c_optIncAnimations, m_data->m_includeAnimationCbx->isChecked() );
	const QString animTake = m_data->m_animationTakeCmb->currentText();
	settings->setStringValue( c_optTake, animTake != tr( c_none ) ? animTake : QString() );

	// Geometry
	settings->setBoolValue( c_optIncPolygonSets, m_data->m_includePolygonSetsCbx->isChecked() );
	settings->setBoolValue( c_optIncPolygonGroups, m_data->m_includePolygonGroupsCbx->isChecked() );

	// Custom Data
	settings->setBoolValue( c_optStudioNodeNamesLabels, m_data->m_studioNodeNameLabelCbx->isChecked() );
	settings->setBoolValue( c_optStudioPresentation, m_data->m_studioPresentationCbx->isChecked() );
	settings->setBoolValue( c_optStudioNodeSelectionMap, m_data->m_studioSelectionMapCbx->isChecked() );
	settings->setBoolValue( c_optStudioSceneIDs, m_data->m_studioSceneIDsCbx->isChecked() );
}

/**
**/
void DzFbxImportFrame::applyChanges()
{
}

/**
**/
void DzFbxImportFrame::resetOptions()
{
	DzFileIOSettings* ioSettings = new DzFileIOSettings();
	m_data->m_importer->getDefaultOptions( ioSettings );
	setOptions( ioSettings, QString() );
}

#include "moc_dzfbximporter.cpp"
