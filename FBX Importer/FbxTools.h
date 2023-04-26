#include <qlist.h>
#include <fbxsdk.h>

//#define Y_TO_Z(a) FbxVector4(a[0], a[2], a[1])
//#define Y_TO_NEGZ(a) FbxVector4(a[0], a[2], -a[1])
//#define NEGY_TO_NEGZ(a) FbxVector4(a[0], -a[2], -a[1])
//#define NEGZ(a) FbxVector4(a[0], a[1], -a[2])

class DzProgress;

class FbxTools
{
public:
	template <typename T> static FbxVector4 transposeYtoZ(const T& a) { return FbxVector4(a[0], a[2], a[1]); }
	template <typename T> static FbxVector4 transposeYtoNegZ(const T& a) { return FbxVector4(a[0], a[2], -a[1]); }
	template <typename T> static FbxVector4 transposeNegYtoNegZ(const T& a) { return FbxVector4(a[0], -a[2], -a[1]); }
	template <typename T> static FbxVector4 transposeToNegZ(const T& a) { return FbxVector4(a[0], a[1], -a[2]); }

	static double getLength(double a, double b);

	static double getLength(double a, double b, double c);

	static double getDistance(FbxVector2 a, FbxVector2 b);

	static double getDistance(FbxVector4 a, FbxVector4 b);

	static double determinant_3x3(FbxVector4* matrix);

	static FbxVector4 calculatePointCloudAverage(FbxMesh* pMesh, QList<int>* pVertexIndexes);

	static FbxVector4 calculatePointCloudCenter(FbxMesh* pMesh, QList<int>* pVertexIndexes, bool bCenterWeight = false);

	static FbxVector4* calculateBoundingVolume(QList<FbxVector4>& pointCloud);

	static FbxVector4* calculateBoundingVolume(FbxMesh* pMesh);

	static FbxVector4* calculateBoundingVolume(FbxMesh* pMesh, QList<int>* pVertexIndexes);

	static void multiplyMatrixInPlace(FbxAMatrix& pMatrix, double pValue);

	static void addToScaleOfMatrixInPlace(FbxAMatrix& pMatrix, double pValue);

	static void addMatrixInPlace(FbxAMatrix& destinationMatrix, const FbxAMatrix& sourceMatrix);

	static FbxAMatrix getPoseMatrix(FbxPose* pPose, int pNodeIndex);

	static FbxAMatrix getAffineMatrix(FbxPose* pPose, int nItemIndex, bool bReturnLocalSpace = false, FbxTime fbxTime = FBXSDK_TIME_INFINITE);

	static FbxAMatrix getAffineMatrix(FbxPose* pPose, FbxNode* pNode, bool bReturnLocalSpace = false, FbxTime fbxTime = FBXSDK_TIME_INFINITE);

	static FbxAMatrix getGeometricAffineMatrix(FbxNode* pNode);

	static bool calculateClusterDeformationMatrix(FbxAMatrix& clusterDeformationMatrix, FbxCluster* pCluster, FbxAMatrix* pGlobalOffsetMatrix, FbxPose* pPose, FbxMesh* pMesh, FbxTime fbxTime = FBXSDK_TIME_INFINITE);

	static bool bakePoseToVertexBufferLinearPathway(FbxVector4* pVertexBuffer, FbxAMatrix* pGlobalOffsetMatrix, FbxPose* pPose, FbxMesh* pMesh, FbxTime fbxTime = FBXSDK_TIME_INFINITE);

	static bool bakePoseToVertexBufferDualQuaternionPathway(FbxVector4* pVertexBuffer, FbxAMatrix* pGlobalOffsetMatrix, FbxPose* pPose, FbxMesh* pMesh, FbxTime fbxTime = FBXSDK_TIME_INFINITE);

	static bool bakePoseToVertexBuffer(FbxVector4* pVertexBuffer, FbxAMatrix* pGlobalOffsetMatrix, FbxPose* pPose, FbxMesh* pMesh, FbxTime pTime = FBXSDK_TIME_INFINITE);

	static FbxAMatrix findPoseMatrixOrIdentity(FbxPose* pPose, FbxNode* pNode);

	static FbxAMatrix findPoseMatrixOrGlobal(FbxPose* pPose, FbxNode* pNode);

	static void removeBindPoses(FbxScene* Scene);

	static FbxPose* saveBindMatrixToPose(FbxScene* pScene, const char* lpPoseName, FbxNode* Argument_pMeshNode = nullptr, bool bAddPose = false);

	static void applyBindPose(FbxScene* pScene, FbxPose* pPose, FbxNode* pNode = nullptr, bool bRecurse = true, bool bClampJoints = false);

	static FbxNode* getRootBone(FbxScene* pScene, bool bRenameRootBone = false, FbxNode* pPreviousBone = nullptr);

	static void detachGeometry(FbxScene* pScene);

	static bool bakePoseToBindMatrix(FbxMesh* pMesh, FbxPose* pPose);

	static bool syncDuplicateBones(FbxScene* lCurrentScene);

	static bool loadAndPoseBelowHeadOnly(QString poseFilePath, FbxScene* lCurrentScene, DzProgress* pProgress = nullptr, bool bConvertToZUp = false);

	static bool loadAndPose(QString poseFilePath, FbxScene* lCurrentScene, DzProgress* pProgress = nullptr, bool bConvertToZUp = false);

	static int convertToZUp(FbxMesh* mesh, FbxNode* rootNode);

	static bool flipAndBakeVertexBuffer(FbxMesh* mesh, FbxNode* rootNode, FbxVector4* vertex_buffer);

	static FbxCluster* findClusterFromNode(FbxNode* pNode);

	static void inspectNode(FbxNode* pNode);

	static bool checkIfChildrenAreBones(FbxNode* pNode);

	static bool isBone(FbxNode* pNode, int skeletonTypeMask = -1);

	static FbxNode* findAssociatedSkeletonRoot(FbxMesh* pMesh);

	static bool getBindMatrixFromCluster(FbxNode* pNode, FbxAMatrix& returnMatrix);

	static bool getBindTranslationFromCluster(FbxNode* pNode, FbxVector4& returnTranslation);

	static bool getBindRotationFromCluster(FbxNode* pNode, FbxVector4& returnRotation);

};


