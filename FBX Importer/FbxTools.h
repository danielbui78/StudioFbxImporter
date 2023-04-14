#include <qlist.h>
#include <fbxsdk.h>

#define Y_TO_Z(a) FbxVector4(a[0], a[2], a[1])
#define Y_TO_NEGZ(a) FbxVector4(a[0], a[2], -a[1])
#define NEGY_TO_NEGZ(a) FbxVector4(a[0], -a[2], -a[1])
#define NEGZ(a) FbxVector4(a[0], a[1], -a[2])

class DzProgress;

double getLength(double a, double b);

double getLength(double a, double b, double c);

double getDistance(FbxVector2 a, FbxVector2 b);

double getDistance(FbxVector4 a, FbxVector4 b);

double determinant_3x3(FbxVector4* matrix);

FbxVector4* CalculateBoundingVolume(QList<FbxVector4>& pointCloud);

FbxVector4* CalculateBoundingVolume(FbxMesh* pMesh);

FbxVector4* CalculateBoundingVolume(FbxMesh* pMesh, QList<int>* pVertexIndexes);

void MultiplyMatrix_InPlace(FbxAMatrix& pMatrix, double pValue);

void AddToScaleOfMatrix_InPlace(FbxAMatrix& pMatrix, double pValue);

void AddMatrix_InPlace(FbxAMatrix& destinationMatrix, const FbxAMatrix& sourceMatrix);

FbxAMatrix GetPoseMatrix(FbxPose* pPose, int pNodeIndex);

FbxAMatrix GetAffineMatrix(FbxPose* pPose, int nItemIndex, bool bReturnLocalSpace = false, FbxTime fbxTime = FBXSDK_TIME_INFINITE);

FbxAMatrix GetAffineMatrix(FbxPose* pPose, FbxNode* pNode, bool bReturnLocalSpace = false, FbxTime fbxTime = FBXSDK_TIME_INFINITE);

FbxAMatrix GetGeometricAffineMatrix(FbxNode* pNode);

bool CalculateClusterDeformationMatrix(FbxAMatrix& clusterDeformationMatrix, FbxCluster* pCluster, FbxAMatrix* pGlobalOffsetMatrix, FbxPose* pPose, FbxMesh* pMesh, FbxTime fbxTime = FBXSDK_TIME_INFINITE);

bool BakePoseToVertexBuffer_LinearPathway(FbxVector4* pVertexBuffer, FbxAMatrix* pGlobalOffsetMatrix, FbxPose* pPose, FbxMesh* pMesh, FbxTime fbxTime = FBXSDK_TIME_INFINITE);

bool BakePoseToVertexBuffer_DualQuaternionPathway(FbxVector4* pVertexBuffer, FbxAMatrix* pGlobalOffsetMatrix, FbxPose* pPose, FbxMesh* pMesh, FbxTime fbxTime = FBXSDK_TIME_INFINITE);

bool BakePoseToVertexBuffer(FbxVector4* pVertexBuffer, FbxAMatrix* pGlobalOffsetMatrix, FbxPose* pPose, FbxMesh* pMesh, FbxTime pTime = FBXSDK_TIME_INFINITE);

FbxAMatrix FindPoseMatrixOrIdentity(FbxPose* pPose, FbxNode* pNode);

FbxAMatrix FindPoseMatrixOrGlobal(FbxPose* pPose, FbxNode* pNode);

void RemoveBindPoses(FbxScene* Scene);

FbxPose* SaveBindMatrixToPose(FbxScene* pScene, const char* lpPoseName, FbxNode* Argument_pMeshNode = nullptr, bool bAddPose = false);

void ApplyBindPose(FbxScene* pScene, FbxPose* pPose, FbxNode* pNode = nullptr, bool bRecurse = true, bool bClampJoints = false);


FbxNode* GetRootBone(FbxScene* pScene, bool bRenameRootBone = false, FbxNode* pPreviousBone = nullptr);

void DetachGeometry(FbxScene* pScene);

bool BakePoseToBindMatrix(FbxMesh* pMesh, FbxPose* pPose);

bool SyncDuplicateBones(FbxScene* lCurrentScene);

bool LoadAndPoseBelowHeadOnly(QString poseFilePath, FbxScene* lCurrentScene, DzProgress* pProgress = nullptr, bool bConvertToZUp = false);

bool LoadAndPose(QString poseFilePath, FbxScene* lCurrentScene, DzProgress* pProgress = nullptr, bool bConvertToZUp = false);

int ConvertToZUp(FbxMesh* mesh, FbxNode* rootNode);

bool FlipAndBakeVertexBuffer(FbxMesh* mesh, FbxNode* rootNode, FbxVector4* vertex_buffer);

FbxCluster* FindClusterFromNode(FbxNode* pNode);

void InspectNode(FbxNode* pNode);

bool CheckIfChildrenAreBones(FbxNode* pNode);

bool IsBone(FbxNode* pNode, int skeletonTypeMask = -1);

FbxNode* FindAssociatedSkeletonRoot(FbxMesh* pMesh);
