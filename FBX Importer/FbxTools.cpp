#define USE_OPENFBX false

#if USE_OPENFBX 
#include "OpenFBXInterface.h"
#endif

#include "FbxTools.h"

#include <fbxsdk.h>
#include <qlist.h>
#include <qmap.h>
#include <dzapp.h>
#include "dzprogress.h"

/////////////////////////////////////////////////////////////////////////////////
/// GEOMETRY FUNCTIONS
double FbxTools::getLength(double a, double b, double c)
{
	double distance = 0;
	double a2 = a * a;
	double b2 = b * b;
	double c2 = c * c;
	distance = sqrt(a2 + b2 + c2);
	return distance;
}

double FbxTools::getLength(double a, double b)
{
	double distance = 0;
	double a2 = a * a;
	double b2 = b * b;
	distance = sqrt(a2 + b2);
	return distance;
}

double FbxTools::getDistance(FbxVector4 a, FbxVector4 b)
{
	FbxVector4 ab = b - a;
	double distance = getLength(ab[0], ab[1], ab[2]);
	return distance;
}

double FbxTools::getDistance(FbxVector2 a, FbxVector2 b)
{
	FbxVector2 ab = b - a;
	double distance = getLength(ab[0], ab[1]);
	return distance;
}

double FbxTools::determinant_3x3(FbxVector4* matrix)
{
	double return_value = 0.0;

	double mat1 = matrix[0][0] * matrix[1][1] * matrix[2][2];
	double mat2 = matrix[0][1] * matrix[1][2] * matrix[2][0];
	double mat3 = matrix[0][2] * matrix[1][0] * matrix[2][1];

	double mat4 = matrix[0][2] * matrix[1][1] * matrix[2][0];
	double mat5 = matrix[0][1] * matrix[1][0] * matrix[2][2];
	double mat6 = matrix[0][0] * matrix[1][2] * matrix[2][1];

	return_value = mat1 + mat2 + mat3 - mat4 - mat5 - mat6;

	return return_value;
}

FbxVector4* FbxTools::calculateBoundingVolume(QList<FbxVector4>& pointCloud)
{

	FbxVector4* result = new FbxVector4[3];

	if (pointCloud.isEmpty())
	{
		return result;
	}

	FbxVector4 cloudCenter;

	FbxVector4 maxBounds = pointCloud[0];
	FbxVector4 minBounds = pointCloud[0];
	FbxVector4 sum(0, 0, 0);
	for (FbxVector4 currentPoint : pointCloud)
	{
		for (int i = 0; i < 3; i++)
		{
			if (maxBounds[i] < currentPoint[i])
			{
				maxBounds[i] = currentPoint[i];
			}
			if (minBounds[i] > currentPoint[i])
			{
				minBounds[i] = currentPoint[i];
			}
		}
		sum += currentPoint;
	}

	FbxVector4 cloudAverage = sum / pointCloud.count();

	cloudCenter[0] = (maxBounds[0] + minBounds[0]) / 2;
	cloudCenter[1] = (maxBounds[1] + minBounds[1]) / 2;
	cloudCenter[2] = (maxBounds[2] + minBounds[2]) / 2;

	FbxVector4 cloudSize;
	cloudSize[0] = abs(maxBounds[0] - minBounds[0]);
	cloudSize[1] = abs(maxBounds[1] - minBounds[1]);
	cloudSize[2] = abs(maxBounds[2] - minBounds[2]);

	result[0] = cloudSize;
	result[1] = cloudCenter;
	result[2] = cloudAverage;

	return result;
}


FbxVector4* FbxTools::calculateBoundingVolume(FbxMesh* pMesh)
{
	FbxVector4* result = new FbxVector4[3];

	FbxVector4 cloudCenter;
	FbxVector4 minBounds;
	FbxVector4 maxBounds;
	FbxVector4 sum(0, 0, 0);

	int numPoints = pMesh->GetControlPointsCount();
	for (int vertex_index = 0; vertex_index < numPoints; vertex_index++)
	{
		FbxVector4 currentPoint = pMesh->GetControlPointAt(vertex_index);
		sum += currentPoint;
		if (vertex_index == 0)
		{
			minBounds = maxBounds = currentPoint;
			continue;
		}
		for (int axis_index = 0; axis_index < 3; axis_index++)
		{
			if (currentPoint[axis_index] < minBounds[axis_index])
				minBounds[axis_index] = currentPoint[axis_index];
			if (currentPoint[axis_index] > maxBounds[axis_index])
				maxBounds[axis_index] = currentPoint[axis_index];
		}
	}

	FbxVector4 cloudAverage = sum / numPoints;

	cloudCenter[0] = (maxBounds[0] + minBounds[0]) / 2;
	cloudCenter[1] = (maxBounds[1] + minBounds[1]) / 2;
	cloudCenter[2] = (maxBounds[2] + minBounds[2]) / 2;

	FbxVector4 cloudSize;
	cloudSize[0] = abs(maxBounds[0] - minBounds[0]);
	cloudSize[1] = abs(maxBounds[1] - minBounds[1]);
	cloudSize[2] = abs(maxBounds[2] - minBounds[2]);

	result[0] = cloudSize;
	result[1] = cloudCenter;
	result[2] = cloudAverage;

	return result;
}

FbxVector4* FbxTools::calculateBoundingVolume(FbxMesh* pMesh, QList<int>* pVertexIndexes)
{
	FbxVector4* result = new FbxVector4[3];

	FbxVector4 cloudAverage;
	FbxVector4 cloudCenter;
	FbxVector4 minBounds;
	FbxVector4 maxBounds;

	bool bFirstElement = true;
	double totalWeights = 0.0;
	for (int vertex_index : (*pVertexIndexes))
	{
		FbxVector4 currentPoint = pMesh->GetControlPointAt(vertex_index);
		if (bFirstElement)
		{
			bFirstElement = false;
			minBounds = maxBounds = currentPoint;
			cloudAverage = currentPoint;
			totalWeights = 1.0;
			continue;
		}
		for (int axis_index = 0; axis_index < 3; axis_index++)
		{
			cloudAverage += currentPoint;
			totalWeights += 1.0;
			if (currentPoint[axis_index] < minBounds[axis_index])
				minBounds[axis_index] = currentPoint[axis_index];
			if (currentPoint[axis_index] > maxBounds[axis_index])
				maxBounds[axis_index] = currentPoint[axis_index];
		}
	}

	cloudAverage = cloudAverage / totalWeights;

	cloudCenter[0] = (maxBounds[0] + minBounds[0]) / 2;
	cloudCenter[1] = (maxBounds[1] + minBounds[1]) / 2;
	cloudCenter[2] = (maxBounds[2] + minBounds[2]) / 2;

	FbxVector4 cloudSize;
	cloudSize[0] = abs(maxBounds[0] - minBounds[0]);
	cloudSize[1] = abs(maxBounds[1] - minBounds[1]);
	cloudSize[2] = abs(maxBounds[2] - minBounds[2]);

	result[0] = cloudSize;
	result[1] = cloudCenter;
	result[2] = cloudAverage;

	return result;
}

FbxVector4 FbxTools::calculatePointCloudAverage(FbxMesh* pMesh, QList<int>* pVertexIndexes)
{

	if (pMesh == nullptr || pVertexIndexes == nullptr || pVertexIndexes->count() <= 0)
	{
		dzApp->log("ERROR: calculatePointCloudCenter recieved invalid inputs");
		return nullptr;
	}

	FbxVector4 cloudCenter = FbxVector4(0, 0, 0);
	double totalWeights = 0.0;
	for (int vertex_index : (*pVertexIndexes))
	{
		FbxVector4 currentPoint = pMesh->GetControlPointAt(vertex_index);
		double currentWeight = 1.0;
		cloudCenter[0] += currentPoint[0];
		cloudCenter[1] += currentPoint[1];
		cloudCenter[2] += currentPoint[2];
		totalWeights += currentWeight;
	}
	cloudCenter[0] = cloudCenter[0] / totalWeights;
	cloudCenter[1] = cloudCenter[1] / totalWeights;
	cloudCenter[2] = cloudCenter[2] / totalWeights;

	return cloudCenter;

}

FbxVector4 FbxTools::calculatePointCloudCenter(FbxMesh* pMesh, QList<int>* pVertexIndexes, bool bCenterWeight)
{

	if (pMesh == nullptr || pVertexIndexes == nullptr || pVertexIndexes->count() <= 0)
	{
		dzApp->log("ERROR: calculatePointCloudCenter recieved invalid inputs");
		return nullptr;
	}

	FbxVector4 cloudCenter = pMesh->GetControlPointAt(pVertexIndexes->first());
	FbxVector4 min_bounds = cloudCenter;
	FbxVector4 max_bounds = cloudCenter;
	for (int vertex_index : (*pVertexIndexes))
	{
		FbxVector4 currentPoint = pMesh->GetControlPointAt(vertex_index);
		for (int i = 0; i < 3; i++)
		{
			if (currentPoint[i] < min_bounds[i]) min_bounds[i] = currentPoint[i];
			if (currentPoint[i] > max_bounds[i]) max_bounds[i] = currentPoint[i];
		}
	}
	double center_weight = 0;
	if (abs(max_bounds[0]) < abs(min_bounds[0]))
		center_weight = max_bounds[0];
	else
		center_weight = min_bounds[0];

	if (bCenterWeight)
		//		cloudCenter[0] = (max_bounds[0] + min_bounds[0] + center_weight) / 3;
		cloudCenter[0] = center_weight;
	else
		cloudCenter[0] = (max_bounds[0] + min_bounds[0]) / 2;
	cloudCenter[1] = (max_bounds[1] + min_bounds[1]) / 2;
	cloudCenter[2] = (max_bounds[2] + min_bounds[2]) / 2;

	FbxVector4 cloudAverage = calculatePointCloudAverage(pMesh, pVertexIndexes);

	return cloudCenter;

}


////////////////////////////////////////////
/// FBX CLUSTER DEFORM FUNCTIONS

// Scale all the elements of a matrix.
void FbxTools::multiplyMatrixInPlace(FbxAMatrix& pMatrix, double pValue)
{
	int i, j;

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			pMatrix[i][j] *= pValue;
		}
	}
}

// Add a value to all the elements in the diagonal of the matrix.
void FbxTools::addToScaleOfMatrixInPlace(FbxAMatrix& pMatrix, double pValue)
{
	for (int i = 0; i < 4; i++)
	{
		pMatrix[i][i] += pValue;
	}
}

// Sum two matrices element by element
void FbxTools::addMatrixInPlace(FbxAMatrix& destinationMatrix, const FbxAMatrix& sourceMatrix)
{
	int i, j;

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			destinationMatrix[i][j] += sourceMatrix[i][j];
		}
	}
}

// Get the matrix of the given pose
FbxAMatrix FbxTools::getPoseMatrix(FbxPose* pPose, int pNodeIndex)
{
	FbxAMatrix lPoseMatrix;
	FbxMatrix lMatrix = pPose->GetMatrix(pNodeIndex);

	memcpy((double*)lPoseMatrix, (double*)lMatrix, sizeof(lMatrix.mData));

	return lPoseMatrix;
}

FbxAMatrix FbxTools::getAffineMatrix(FbxPose* pPose, int nItemIndex, bool bReturnLocalSpace, FbxTime fbxTime)
{
	/////////////////////
	// DEFAULT CASES: Return global or local pose matrix (with matching bReturnLocalSpace)
	////////////////////////
	FbxAMatrix returnMatrix;
	FbxMatrix tempMatrix = pPose->GetMatrix(nItemIndex);
	memcpy(&returnMatrix, &tempMatrix, sizeof(tempMatrix.mData));

	/////////////////////////
	// OTHER CONDITIONS
	/////////////////////////
	if (pPose->IsLocalMatrix(nItemIndex) == true && bReturnLocalSpace == false)
	{
		FbxNode* pParentNode = pPose->GetNode(nItemIndex)->GetParent();
		if (pParentNode)
		{
			FbxAMatrix parentMatrix;
			int nParentIndex = pPose->Find(pParentNode);
			if (nParentIndex > -1)
			{
				parentMatrix = getAffineMatrix(pPose, nParentIndex, bReturnLocalSpace, fbxTime);
			}
			else
			{
				parentMatrix = pParentNode->EvaluateGlobalTransform(fbxTime);
			}
			FbxAMatrix tempMatrix2 = parentMatrix * returnMatrix;
			returnMatrix = tempMatrix2;
		}
	}
	else if (pPose->IsLocalMatrix(nItemIndex) == false && bReturnLocalSpace == true)
	{
		FbxNode* pParentNode = pPose->GetNode(nItemIndex)->GetParent();
		if (pParentNode)
		{
			FbxAMatrix parentMatrix;
			int nParentIndex = pPose->Find(pParentNode);
			if (nParentIndex > -1)
			{
				parentMatrix = getAffineMatrix(pPose, nParentIndex, bReturnLocalSpace, fbxTime);
			}
			else
			{
				parentMatrix = pParentNode->EvaluateGlobalTransform(fbxTime);
			}
			FbxAMatrix tempMatrix2 = parentMatrix.Inverse() * returnMatrix;
			returnMatrix = tempMatrix2;
		}
	}
	////////
	// NOTE: DEFAULT CASES ALREADY ASSIGNED ABOVE
	////////

	return returnMatrix;
}

// Return matrix of pNode, using pPose if it is present, using WS matrix by default, using time infinite by default
FbxAMatrix FbxTools::getAffineMatrix(FbxPose* pPose, FbxNode* pNode, bool bReturnLocalSpace, FbxTime fbxTime)
{
	FbxAMatrix returnMatrix;

	if (pPose != nullptr)
	{
		int nodeIndex = pPose->Find(pNode);
		if (nodeIndex == -1)
		{
			QString sActiveNodeName(pNode->GetName());
			for (int i = 0; i < pPose->GetCount(); i++)
			{
				FbxNode* current_node = pPose->GetNode(i);
				QString sCurrentNodeName(current_node->GetName());
				if (sCurrentNodeName.contains(sActiveNodeName) == true)
				{
					nodeIndex = i;
					break;
				}
			}
		}

		if (nodeIndex > -1)
		{
			returnMatrix = getAffineMatrix(pPose, nodeIndex, bReturnLocalSpace);
			return returnMatrix;
		}
	}
	if (bReturnLocalSpace == false)
	{
		returnMatrix = pNode->EvaluateGlobalTransform(fbxTime);
	}
	else
	{
		returnMatrix = pNode->EvaluateLocalTransform(fbxTime);
	}
	return returnMatrix;
}

FbxAMatrix FbxTools::getGeometricAffineMatrix(FbxNode* pNode)
{
	FbxVector4 t = pNode->GetGeometricTranslation(FbxNode::eSourcePivot);
	FbxVector4 r = pNode->GetGeometricRotation(FbxNode::eSourcePivot);
	FbxVector4 s = pNode->GetGeometricScaling(FbxNode::eSourcePivot);

	FbxAMatrix returnMatrix(t, r, s);

	return returnMatrix;
}

bool FbxTools::calculateClusterDeformationMatrix(FbxAMatrix& clusterDeformationMatrix, FbxCluster* pCluster, FbxAMatrix* pGlobalOffsetMatrix, FbxPose* pPose, FbxMesh* pMesh, FbxTime fbxTime)
{
	bool bResult = false;
	// if cluster link mode is eAdditive
	if (pCluster->GetLinkMode() == FbxCluster::eAdditive)
	{
		FbxAMatrix clusterBindMatrix_x_Geo;
		pCluster->GetTransformMatrix(clusterBindMatrix_x_Geo);
		FbxAMatrix meshGeoMatrix = getGeometricAffineMatrix(pMesh->GetNode());
		clusterBindMatrix_x_Geo *= meshGeoMatrix;

		// associate matrix
		FbxAMatrix associateModelMatrix;
		pCluster->GetTransformAssociateModelMatrix(associateModelMatrix);

		FbxNode* pAssociateMesh = pCluster->GetAssociateModel();
		FbxAMatrix associateGeoMatrix = getGeometricAffineMatrix(pAssociateMesh);
		FbxAMatrix associateModelPosedMatrix = getAffineMatrix(pPose, pCluster->GetAssociateModel(), false, fbxTime);

		FbxAMatrix clusterPosedMatrix = getAffineMatrix(pPose, pCluster->GetLink(), false, fbxTime);

		FbxAMatrix clusterLinkBindMatrix_x_Geo;
		pCluster->GetTransformLinkMatrix(clusterLinkBindMatrix_x_Geo);
		FbxAMatrix clusterLinkGeoMatrix = getGeometricAffineMatrix(pCluster->GetLink());
		clusterLinkBindMatrix_x_Geo *= clusterLinkGeoMatrix;

		/////// Compute the shift of the link relative to the reference.
		// reference_inverse * associate * associate_geo_inverse * link_geo * link_geo_inverse * reference
		clusterDeformationMatrix = clusterBindMatrix_x_Geo.Inverse() * associateModelMatrix * associateModelPosedMatrix.Inverse() *
			clusterPosedMatrix * clusterLinkBindMatrix_x_Geo.Inverse() * clusterBindMatrix_x_Geo;
		bResult = true;
	}
	else
	{
		FbxAMatrix clusterPosedMatrix = getAffineMatrix(pPose, pCluster->GetLink(), false, fbxTime);

		FbxAMatrix clusterLinkBindMatrix_x_Geo;
		pCluster->GetTransformLinkMatrix(clusterLinkBindMatrix_x_Geo);

		FbxAMatrix clusterBindMatrix_x_Geo;
		pCluster->GetTransformMatrix(clusterBindMatrix_x_Geo);
		FbxAMatrix meshGeoMatrix = getGeometricAffineMatrix(pMesh->GetNode());
		clusterBindMatrix_x_Geo *= meshGeoMatrix;

		// relative_current_inverse * relative_initial
		clusterDeformationMatrix = pGlobalOffsetMatrix->Inverse() * clusterPosedMatrix *
			clusterLinkBindMatrix_x_Geo.Inverse() * clusterBindMatrix_x_Geo;
		bResult = true;
	}

	return bResult;
}

bool FbxTools::bakePoseToVertexBufferLinearPathway(FbxVector4* pVertexBuffer, FbxAMatrix* pGlobalOffsetMatrix, FbxPose* pPose, FbxMesh* pMesh, FbxTime fbxTime)
{
	bool bResult = false;
	// get cluster link mode
	FbxSkin* pSkinDeformer = (FbxSkin*)pMesh->GetDeformer(0, FbxDeformer::eSkin);
	FbxCluster* pCluster = pSkinDeformer->GetCluster(0);
	FbxCluster::ELinkMode clusterMode = pCluster->GetLinkMode();

	int numVerts = pMesh->GetControlPointsCount();
	// prepare cluster matrix buffer (one matrix per vertex)
	FbxAMatrix* pMatrixBuffer = new FbxAMatrix[numVerts];
	memset(pMatrixBuffer, 0, numVerts * sizeof(FbxAMatrix));
	// prepare cluster weight buffer (one weight per vertex)
	double* pWeightBuffer = new double[numVerts];
	memset(pWeightBuffer, 0, numVerts * sizeof(double));
	// if addtive cluster mode, set each matrix in matrix buffer to identity
	if (clusterMode == FbxCluster::eAdditive)
	{
		for (int matrixIndex = 0; matrixIndex < numVerts; matrixIndex++)
		{
			pMatrixBuffer[matrixIndex].SetIdentity();
		}
	}

	// for each cluster in each skindeformer of mesh, calc matrix transform and weights per vertex
	int numSkinDeformers = pMesh->GetDeformerCount(FbxSkin::eSkin);
	for (int skinIndex = 0; skinIndex < numSkinDeformers; skinIndex++)
	{
		FbxSkin* pCurrentSkinDeformer = (FbxSkin*)pMesh->GetDeformer(skinIndex, FbxSkin::eSkin);
		int numClusters = pCurrentSkinDeformer->GetClusterCount();
		for (int clusterIndex = 0; clusterIndex < numClusters; clusterIndex++)
		{
			if (pCurrentSkinDeformer->GetCluster(clusterIndex)->GetLink() == nullptr)
			{
				//printf("DEBUG: cluster is not linked to any bone, skipping cluster[%i]", clusterIndex);
				continue;
			}
			FbxCluster* pCurrentCluster = pCurrentSkinDeformer->GetCluster(clusterIndex);

			FbxAMatrix clusterTransformMatrix;
			if (calculateClusterDeformationMatrix(clusterTransformMatrix, pCurrentCluster, pGlobalOffsetMatrix, pPose, pMesh, fbxTime) == false)
			{
				//printf("ERROR: unable to calculate cluster deformation matrix, skipping cluster[%i]", clusterIndex);
				continue;
			}
			// each cluster has a list of indexes into the global vertex index buffer
			// localIndex == offset into each cluster's buffer of vertex indexes
			// globalIndex == offset into the global vertex buffer
			int numLocalIndexes = pCurrentCluster->GetControlPointIndicesCount();
			for (int localIndex = 0; localIndex < numLocalIndexes; localIndex++)
			{
				int globalIndex = pCurrentCluster->GetControlPointIndices()[localIndex];
				if (globalIndex >= numVerts)
				{
					//printf("ERROR: global vertex index is out of range of global vertex buffer: globalIndex=[%i]", globalIndex);
					continue;
				}
				double fWeightOfVertex = pCurrentCluster->GetControlPointWeights()[localIndex];
				FbxAMatrix weightedTransformMatrix = clusterTransformMatrix;
				multiplyMatrixInPlace(weightedTransformMatrix, fWeightOfVertex);
				if (clusterMode == FbxCluster::eAdditive)
				{
					addToScaleOfMatrixInPlace(weightedTransformMatrix, 1 - fWeightOfVertex);
					pMatrixBuffer[globalIndex] = weightedTransformMatrix * pMatrixBuffer[globalIndex];
					pWeightBuffer[globalIndex] = 1.0;
				}
				else
				{
					addMatrixInPlace(pMatrixBuffer[globalIndex], weightedTransformMatrix);
					pWeightBuffer[globalIndex] += fWeightOfVertex;
				}
			}

		}
	}

	// apply weight * matrix transform to each vertex
	for (int globalIndex = 0; globalIndex < numVerts; globalIndex++)
	{
		FbxVector4 sourceVertex = pVertexBuffer[globalIndex];
		FbxVector4 finalTargetVertex;
		double fVertexWeight = pWeightBuffer[globalIndex];
		if (fVertexWeight != 0.0)
		{
			FbxVector4 intermediateVertexValue = pMatrixBuffer[globalIndex].MultT(sourceVertex);
			if (clusterMode == FbxCluster::eNormalize)
			{
				finalTargetVertex = intermediateVertexValue / fVertexWeight;
			}
			else if (clusterMode == FbxCluster::eTotalOne)
			{
				finalTargetVertex = intermediateVertexValue + sourceVertex * (1 - fVertexWeight);
			}
			else
			{
				finalTargetVertex = intermediateVertexValue;
			}
			pVertexBuffer[globalIndex] = finalTargetVertex;
		}
	}
	bResult = true;

	// cleanup buffers
	delete[] pMatrixBuffer;
	delete[] pWeightBuffer;

	return bResult;
}

bool FbxTools::bakePoseToVertexBufferDualQuaternionPathway(FbxVector4* pVertexBuffer, FbxAMatrix* pGlobalOffsetMatrix, FbxPose* pPose, FbxMesh* pMesh, FbxTime fbxTime)
{
	bool bResult = false;
	// get cluster link mode
	FbxSkin* pSkinDeformer = (FbxSkin*)pMesh->GetDeformer(0, FbxDeformer::eSkin);
	FbxCluster* pCluster = pSkinDeformer->GetCluster(0);
	FbxCluster::ELinkMode clusterMode = pCluster->GetLinkMode();

	int numVerts = pMesh->GetControlPointsCount();
	// prepare dual-quaternion buffer (one DQ per vertex)
	FbxDualQuaternion* pDualQuaternionBuffer = new FbxDualQuaternion[numVerts];
	memset(pDualQuaternionBuffer, 0, numVerts * sizeof(FbxDualQuaternion));
	// prepare cluster weight buffer (one weight per vertex)
	double* pWeightBuffer = new double[numVerts];
	memset(pWeightBuffer, 0, numVerts * sizeof(double));

	// for each cluster of each skindeformer of mesh
	int numSkinDeformers = pMesh->GetDeformerCount(FbxSkin::eSkin);
	for (int skinIndex = 0; skinIndex < numSkinDeformers; skinIndex++)
	{
		FbxSkin* pCurrentSkinDeformer = (FbxSkin*)pMesh->GetDeformer(skinIndex, FbxSkin::eSkin);
		int numClusters = pCurrentSkinDeformer->GetClusterCount();
		for (int clusterIndex = 0; clusterIndex < numClusters; clusterIndex++)
		{
			if (pCurrentSkinDeformer->GetCluster(clusterIndex)->GetLink() == nullptr)
			{
				//printf("DEBUG: cluster is not linked to any bone, skipping cluster[%i]", clusterIndex);
				continue;
			}
			FbxCluster* pCurrentCluster = pCurrentSkinDeformer->GetCluster(clusterIndex);

			FbxAMatrix clusterTransformMatrix;
			if (calculateClusterDeformationMatrix(clusterTransformMatrix, pCurrentCluster, pGlobalOffsetMatrix, pPose, pMesh, fbxTime) == false)
			{
				//printf("ERROR: unable to calculate cluster deformation matrix, skipping cluster[%i]", clusterIndex);
				continue;
			}
			// compute DQ deformation and weight for each vertex
			FbxQuaternion componentQuaternion = clusterTransformMatrix.GetQ();
			FbxVector4 componentTranslation = clusterTransformMatrix.GetT();
			FbxDualQuaternion clusterDualQuaternion(componentQuaternion, componentTranslation);

			// each cluster has a list of indexes into the global vertex index buffer
			// localIndex == offset into each cluster's buffer of vertex indexes
			// globalIndex == offset into the global vertex buffer
			int numLocalIndexes = pCurrentCluster->GetControlPointIndicesCount();
			for (int localIndex = 0; localIndex < numLocalIndexes; localIndex++)
			{
				int globalIndex = pCurrentCluster->GetControlPointIndices()[localIndex];
				if (globalIndex >= numVerts)
				{
					//printf("ERROR: global vertex index is out of range of global vertex buffer: globalIndex=[%i]", globalIndex);
					continue;
				}
				double fWeightOfVertex = pCurrentCluster->GetControlPointWeights()[localIndex];
				if (fWeightOfVertex != 0.0)
				{
					FbxDualQuaternion weightedDualQuaternion = clusterDualQuaternion * fWeightOfVertex;
					if (clusterMode == FbxCluster::eAdditive)
					{
						pDualQuaternionBuffer[globalIndex] = weightedDualQuaternion;
						pWeightBuffer[globalIndex] = 1.0;
					}
					else
					{
						pWeightBuffer[globalIndex] += fWeightOfVertex;
						if (clusterIndex == 0)
						{
							pDualQuaternionBuffer[globalIndex] = weightedDualQuaternion;
						}
						else
						{
							FbxQuaternion quaternionA = pDualQuaternionBuffer[globalIndex].GetFirstQuaternion();
							FbxQuaternion quaternionB = weightedDualQuaternion.GetFirstQuaternion();
							double fSign = quaternionA.DotProduct(quaternionB);
							if (fSign >= 0.0)
							{
								pDualQuaternionBuffer[globalIndex] += weightedDualQuaternion;
							}
							else
							{
								pDualQuaternionBuffer[globalIndex] -= weightedDualQuaternion;
							}
						}
					}

				}

			}


		}
	}

	// apply weighted DQ deformation, based on cluster link mode
	for (int globalIndex = 0; globalIndex < numVerts; globalIndex++)
	{
		FbxVector4 sourceVertex = pVertexBuffer[globalIndex];
		FbxVector4 finalTargetVertex = sourceVertex;
		double fVertexWeight = pWeightBuffer[globalIndex];
		if (fVertexWeight != 0.0)
		{
			pDualQuaternionBuffer[globalIndex].Normalize();
			FbxVector4 intermediateVertexValue = pDualQuaternionBuffer[globalIndex].Deform(finalTargetVertex);
			if (clusterMode == FbxCluster::eNormalize)
			{
				finalTargetVertex = intermediateVertexValue / fVertexWeight;
			}
			else if (clusterMode == FbxCluster::eTotalOne)
			{
				finalTargetVertex = intermediateVertexValue + sourceVertex * (1.0 - fVertexWeight);
			}
			else
			{
				finalTargetVertex = intermediateVertexValue;
			}

			pVertexBuffer[globalIndex] = finalTargetVertex;
		}
	}
	bResult = true;

	// cleanup buffers
	delete[] pDualQuaternionBuffer;
	delete[] pWeightBuffer;

	return bResult;
}

bool FbxTools::bakePoseToVertexBuffer(FbxVector4* pVertexBuffer, FbxAMatrix* pGlobalOffsetMatrix, FbxPose* pPose, FbxMesh* pMesh, FbxTime pTime)
{
	bool bResult = false;
	// get skin deformer for mesh
	FbxSkin* pSkinDeformer = (FbxSkin*)pMesh->GetDeformer(0, FbxDeformer::eSkin);
	FbxSkin::EType skinningType = pSkinDeformer->GetSkinningType();

	// choose linear, dual-quaternion or blend pathways
	switch (skinningType)
	{
	case FbxSkin::eLinear:
	case FbxSkin::eRigid:
		bResult = bakePoseToVertexBufferLinearPathway(pVertexBuffer, pGlobalOffsetMatrix, pPose, pMesh, pTime);
		break;
	case FbxSkin::eDualQuaternion:
		bResult = bakePoseToVertexBufferDualQuaternionPathway(pVertexBuffer, pGlobalOffsetMatrix, pPose, pMesh, pTime);
		break;
	case FbxSkin::eBlend:
		// create temp vertex buffers to compute linear & quaternion pathways
		// linear
		int numVerts = pMesh->GetControlPointsCount();
		FbxVector4* pVertexBuffer_Linear = new FbxVector4[numVerts];
		memcpy(pVertexBuffer_Linear, pMesh->GetControlPoints(), numVerts * sizeof(FbxVector4));
		bakePoseToVertexBufferLinearPathway(pVertexBuffer_Linear, pGlobalOffsetMatrix, pPose, pMesh, pTime);
		// dual-quaternion
		FbxVector4* pVertexBuffer_DQ = new FbxVector4[numVerts];
		memcpy(pVertexBuffer_DQ, pMesh->GetControlPoints(), numVerts * sizeof(FbxVector4));
		bakePoseToVertexBufferDualQuaternionPathway(pVertexBuffer_DQ, pGlobalOffsetMatrix, pPose, pMesh, pTime);
		// linear-interpolate between the two buffer results
		int numBlendWeights = pSkinDeformer->GetControlPointIndicesCount();
		double* pBlendWeightBuffer = pSkinDeformer->GetControlPointBlendWeights();
		for (int nVertexIndex = 0; nVertexIndex < numBlendWeights; nVertexIndex++)
		{
			double fBlendWeight = pBlendWeightBuffer[nVertexIndex];
			FbxVector4 linearResult = pVertexBuffer_Linear[nVertexIndex];
			FbxVector4 dqResult = pVertexBuffer_DQ[nVertexIndex];
			pVertexBuffer[nVertexIndex] = (linearResult * fBlendWeight) + (dqResult * (1 - fBlendWeight));
		}
		// cleanup buffers
		delete[] pVertexBuffer_Linear;
		delete[] pVertexBuffer_DQ;
		bResult = true;
		break;
	}

	return bResult;
}


////////////////////////////////////////////
/// FBX POSE FUNCTIONS
FbxAMatrix FbxTools::findPoseMatrixOrIdentity(FbxPose* pPose, FbxNode* pNode)
{
	FbxAMatrix returnMatrix;

	int nodeIndex = pPose->Find(pNode);
	if (nodeIndex > -1)
	{
		returnMatrix = getAffineMatrix(pPose, nodeIndex);
	}
	else
	{
		returnMatrix.SetIdentity();
	}

	return returnMatrix;
}

FbxAMatrix FbxTools::findPoseMatrixOrGlobal(FbxPose* pPose, FbxNode* pNode)
{
	FbxAMatrix returnMatrix;

	int nodeIndex = pPose->Find(pNode);
	if (nodeIndex > -1)
	{
		returnMatrix = getAffineMatrix(pPose, nodeIndex);
	}
	else
	{
		returnMatrix = pNode->EvaluateGlobalTransform(FBXSDK_TIME_INFINITE);
	}

	return returnMatrix;
}

void FbxTools::removeBindPoses(FbxScene* Scene)
{
	QList<int> poseIndexesToDelete;
	int numPoses = Scene->GetPoseCount();
	for (int PoseIndex = numPoses - 1; PoseIndex >= 0; --PoseIndex)
	{
		FbxPose* pPose = Scene->GetPose(PoseIndex);
		if (pPose->IsBindPose())
		{
			//			ApplyPose(Scene, pPose);
			for (int nGeoIndex = 0; nGeoIndex < Scene->GetGeometryCount(); nGeoIndex++)
			{
				FbxMesh* mesh = (FbxMesh*)Scene->GetGeometry(0);
				FbxVector4* vertex_buffer = mesh->GetControlPoints();
				//				ComputeSkinDeformation(GetGlobalPosition(mesh->GetNode(), FbxTime(0), pPose), mesh, FbxTime(0), vertex_buffer, NULL);
			}
			poseIndexesToDelete.append(PoseIndex);

		}
	}

	for (int i : poseIndexesToDelete)
	{
		Scene->RemovePose(i);
	}

}

FbxPose* FbxTools::saveBindMatrixToPose(FbxScene* pScene, const char* lpPoseName, FbxNode* Argument_pMeshNode, bool bAddPose)
{
	FbxPose* pNewBindPose = FbxPose::Create(pScene->GetFbxManager(), lpPoseName);

	QList<FbxNode*> todoList;
	FbxNode* pRootNode = pScene->GetRootNode();
	if (Argument_pMeshNode != nullptr)
	{
		pRootNode = Argument_pMeshNode;
	}
	todoList.append(pRootNode);

	while (todoList.isEmpty() == false)
	{
		FbxNode* pCurrentMeshNode = todoList.front();
		todoList.pop_front();
		const char* lpCurrentMeshNodeName = pCurrentMeshNode->GetName();
		FbxGeometry* pGeometry = static_cast<FbxGeometry*>(pCurrentMeshNode->GetMesh());
		if (pGeometry)
		{
			for (int nDeformerIndex = 0; nDeformerIndex < pGeometry->GetDeformerCount(); ++nDeformerIndex)
			{
				FbxSkin* pSkin = static_cast<FbxSkin*>(pGeometry->GetDeformer(nDeformerIndex));
				if (pSkin)
				{
					for (int nClusterIndex = 0; nClusterIndex < pSkin->GetClusterCount(); ++nClusterIndex)
					{
						FbxCluster* pCluster = pSkin->GetCluster(nClusterIndex);
						FbxNode* pClusterBone = pCluster->GetLink();
						const char* pBoneName = pClusterBone->GetName();
						FbxAMatrix bindMatrix;
						pCluster->GetTransformLinkMatrix(bindMatrix);
						pNewBindPose->Add(pClusterBone, bindMatrix, false);

						if (QString(pBoneName).contains("lShldrBend", Qt::CaseInsensitive))
						{
							FbxVector4 rotation = bindMatrix.GetR();
							//printf("nop");
						}
						if (QString(pBoneName).contains("lForearmBend", Qt::CaseInsensitive))
						{
							FbxVector4 rotation = bindMatrix.GetR();
							//printf("nop");
						}

					}
				}
			}
		}
		for (int nChildIndex = 0; nChildIndex < pCurrentMeshNode->GetChildCount(); ++nChildIndex)
		{
			FbxNode* pChildBone = pCurrentMeshNode->GetChild(nChildIndex);
			todoList.push_back(pChildBone);
		}
	}
	if (bAddPose)
	{
		pScene->AddPose(pNewBindPose);
	}
	return pNewBindPose;
}

void FbxTools::applyBindPose(FbxScene* pScene, FbxPose* pPose, FbxNode* pNode, bool bRecurse, bool bClampJoints)
{
	// loop and perform for each node starting with root node
	if (pNode == nullptr)
	{
		pNode = pScene->GetRootNode();
	}
	const char* lpNodeName = pNode->GetName();

	FbxAMatrix poseMatrix;
	FbxAMatrix parentMatrix;
	FbxAMatrix localMatrix;
	// find node in main scene
	FbxNode* pParentNode = pNode->GetParent();
	if (pParentNode == NULL)
	{
		localMatrix = findPoseMatrixOrGlobal(pPose, pNode);
	}
	else
	{
		const char* lpParentNodeName = pParentNode->GetName();
		parentMatrix = findPoseMatrixOrGlobal(pPose, pParentNode);

		poseMatrix = findPoseMatrixOrGlobal(pPose, pNode);

		localMatrix = parentMatrix.Inverse() * poseMatrix;
	}

	if (bClampJoints)
	{
		//		ClampTransform(pNode, &localMatrix);
	}

	//// rotation order
	FbxVector4 correctRotation = localMatrix.GetR();
	FbxRotationOrder rotationOrderFixer(pNode->RotationOrder.Get());
	rotationOrderFixer.M2V(correctRotation, localMatrix);

	pNode->SetPreRotation(FbxNode::EPivotSet::eSourcePivot, FbxVector4(0, 0, 0));
	pNode->SetPostRotation(FbxNode::EPivotSet::eSourcePivot, FbxVector4(0, 0, 0));
	pNode->SetRotationOffset(FbxNode::EPivotSet::eSourcePivot, FbxVector4(0, 0, 0));

	pNode->LclTranslation.Set(localMatrix.GetT());
	pNode->LclRotation.Set(correctRotation);
	pNode->LclScaling.Set(localMatrix.GetS());


	if (QString(lpNodeName).contains("lForearmBend", Qt::CaseInsensitive))
	{
		FbxVector4 localRot = localMatrix.GetR();
		FbxVector4 poseRot = poseMatrix.GetR();
		FbxVector4 parentRot = parentMatrix.GetR();
		const char* lpParentName = pParentNode->GetName();

		//printf("nop");
	}

	if (bRecurse == false)
		return;

	// apply to all children
	for (int childIndex = 0; childIndex < pNode->GetChildCount(); childIndex++)
	{
		FbxNode* pChildNode = pNode->GetChild(childIndex);
		applyBindPose(pScene, pPose, pChildNode, bRecurse, bClampJoints);
	}

}


bool FbxTools::bakePoseToBindMatrix(FbxMesh* pMesh, FbxPose* pPose)
{
	// for each cluster,
	// get link node
	// look up link node in pose
	// apply pose matrix to bindmatrix with SetTransformLinkMatrix

	int numSkinDeformers = pMesh->GetDeformerCount(FbxSkin::eSkin);
	for (int skinIndex = 0; skinIndex < numSkinDeformers; skinIndex++)
	{
		FbxSkin* pCurrentSkinDeformer = (FbxSkin*)pMesh->GetDeformer(skinIndex, FbxSkin::eSkin);
		int numClusters = pCurrentSkinDeformer->GetClusterCount();
		for (int clusterIndex = 0; clusterIndex < numClusters; clusterIndex++)
		{
			FbxCluster* pCurrentCluster = pCurrentSkinDeformer->GetCluster(clusterIndex);
			FbxNode* clusterBone = pCurrentCluster->GetLink();
			if (clusterBone == nullptr)
			{
				//printf("DEBUG: cluster is not linked to any bone, skipping cluster[%i]", clusterIndex);
				continue;
			}

			bool bNoPoseBone = false;
			if (pPose != nullptr)
			{
				const char* lpBoneName = clusterBone->GetName();
				QString sSearchName(lpBoneName);
				int poseNodeIndex = -1;
				for (int i = 0; i < pPose->GetCount(); i++)
				{
					FbxNode* current_node = pPose->GetNode(i);
					QString sCurrentNodeName(current_node->GetName());
					if (sCurrentNodeName == sSearchName)
					{
						poseNodeIndex = i;
						break;
					}
				}
				if (poseNodeIndex != -1)
				{
					assert(pPose->IsLocalMatrix(poseNodeIndex) == false);
					FbxAMatrix poseMatrix = getPoseMatrix(pPose, poseNodeIndex);
					pCurrentCluster->SetTransformLinkMatrix(poseMatrix);
				}
				else
				{
					dzApp->log(QString("ERROR: bakePoseToBindMatrix() could not find cluster bone: %1 in pose").arg(sSearchName));
					bNoPoseBone = true;
				}
			}
			
			if (pPose == nullptr || bNoPoseBone == true)
			{
				FbxAMatrix poseMatrix = getAffineMatrix(nullptr, clusterBone);
				pCurrentCluster->SetTransformLinkMatrix(poseMatrix);
			}

		}
	}

	return true;
}



/////////////////////////////////////////////////////////////////////////////////
/// FBX SCENE FUNCTIONS
FbxNode* FbxTools::getRootBone(FbxScene* pScene, bool bRenameRootBone, FbxNode* pPreviousBone)
{
	QList<FbxNode*> todoList;

	// Find the root bone.  There should only be one bone off the scene root
	FbxNode* pRootNode = pScene->GetRootNode();
	FbxNode* pRootBone = nullptr;
	int rootChildCount = pRootNode->GetChildCount();
	int rootBoneCount = 0;
	bool bFoundPrevious = false;
	for (int nChildIndex = 0; nChildIndex < rootChildCount; ++nChildIndex)
	{
		FbxNode* pChildNode = pRootNode->GetChild(nChildIndex);
		if (pPreviousBone != nullptr && bFoundPrevious == false)
		{
			if (pChildNode == pPreviousBone)
				bFoundPrevious = true;
			continue;
		}
		FbxNodeAttribute* pAttr = pChildNode->GetNodeAttribute();
		if (pAttr && pAttr->GetAttributeType() == FbxNodeAttribute::eSkeleton)
		{
			rootBoneCount++;
			pRootBone = pChildNode;
			const char* lpRootBoneName = pRootBone->GetName();
			if (bRenameRootBone)
			{
				pRootBone->SetName("root");
				pAttr->SetName("root");
			}
			break;
		}
		todoList.append(pChildNode);
	}

	// if first layer failed, search each successive layer
	if (pRootBone == nullptr)
	{
		while (todoList.isEmpty() == false)
		{
			if (pRootBone)
			{
				break;
			}
			FbxNode* pNode = todoList.front();
			todoList.pop_front();
			int nChildCount = pNode->GetChildCount();
			for (int nChildIndex = 0; nChildIndex < nChildCount; nChildIndex++)
			{
				FbxNode* pChildNode = pNode->GetChild(nChildIndex);
				if (pPreviousBone != nullptr && bFoundPrevious == false)
				{
					if (pChildNode == pPreviousBone)
						bFoundPrevious = true;
					continue;
				}
				FbxNodeAttribute* pAttr = pChildNode->GetNodeAttribute();
				if (pAttr && pAttr->GetAttributeType() == FbxNodeAttribute::eSkeleton)
				{
					rootBoneCount++;
					pRootBone = pChildNode;
					const char* lpRootBoneName = pRootBone->GetName();
					if (bRenameRootBone)
					{
						pRootBone->SetName("root");
						pAttr->SetName("root");
					}
					break;
				}
				todoList.append(pChildNode);
			}
		}
	}

	return pRootBone;
}

void FbxTools::detachGeometry(FbxScene* pScene)
{
	FbxNode* RootNode = pScene->GetRootNode();

	// Detach geometry from the skeleton
	for (int NodeIndex = 0; NodeIndex < pScene->GetNodeCount(); ++NodeIndex)
	{
		FbxNode* SceneNode = pScene->GetNode(NodeIndex);
		if (SceneNode == nullptr)
		{
			continue;
		}
		FbxGeometry* NodeGeometry = static_cast<FbxGeometry*>(SceneNode->GetMesh());
		if (NodeGeometry)
		{
			if (SceneNode->GetParent() &&
				SceneNode->GetParent()->GetNodeAttribute() &&
				SceneNode->GetParent()->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eSkeleton)
			{
				SceneNode->GetParent()->RemoveChild(SceneNode);
				RootNode->AddChild(SceneNode);
			}
		}
	}
}

bool FbxTools::syncDuplicateBones(FbxScene* lCurrentScene)
{
	// for each bone with .001, sync with original bone
	for (int i = 0; i < lCurrentScene->GetNodeCount(); i++)
	{
		FbxNode* pBone = lCurrentScene->GetNode(i);
		FbxNodeAttribute* Attr = pBone->GetNodeAttribute();
		if (Attr && Attr->GetAttributeType() == FbxNodeAttribute::eSkeleton)
		{
			const char* lpBoneName = pBone->GetName();
			QString sBoneName(lpBoneName);
			if (sBoneName.contains(".001"))
			{
				QString sOrigBoneName = QString(sBoneName).replace(".001", "");
				FbxNode* pOrigBone = lCurrentScene->FindNodeByName(sOrigBoneName.toLocal8Bit().constData());
				if (pOrigBone)
				{
					//pBone->Copy(*pOrigBone);
					pBone->LclRotation.Set(pOrigBone->LclRotation.Get());
					pBone->LclScaling.Set(pOrigBone->LclScaling.Get());
					pBone->LclTranslation.Set(pOrigBone->LclTranslation.Get());
					pBone->PreRotation.Set(pOrigBone->PreRotation.Get());
					pBone->PostRotation.Set(pOrigBone->PostRotation.Get());
					
				}
				else
				{
					dzApp->log(QString("ERROR: syncDuplicateBones(): OrigBone not found for: %1").arg(sBoneName));
				}
			}
		}
	}
	

	return true;
}

#if USE_OPENFBX
bool FbxTools::loadAndPoseBelowHeadOnly(QString poseFilePath, FbxScene* lCurrentScene, DzProgress* pProgress, bool bConvertToZUp)
{
	OpenFBXInterface* openFBX = OpenFBXInterface::GetInterface();

	FbxScene* pPoseScene = openFBX->CreateScene("My Scene");
	if (openFBX->LoadScene(pPoseScene, poseFilePath.toLocal8Bit().data()) == false)
	{
		return false;
	}
	if (pProgress) pProgress->step();

	// make nodename lookup table
	QMap<QString, FbxNode*> lookupTable;
	int numPoseNodes = pPoseScene->GetNodeCount();
	for (int i = 0; i < numPoseNodes; i++)
	{
		FbxNode* pNode = pPoseScene->GetNode(i);
		const char* lpNodeName = pNode->GetName();
		QString sNodeName(lpNodeName);
		lookupTable.insert(sNodeName, pNode);
	}
	if (pProgress) pProgress->step();

	// Convert Pose Scene to Zup
	if (bConvertToZUp)
	{
		FbxMesh* mesh = (FbxMesh*)pPoseScene->GetGeometry(0);
		if (mesh)
		{
			convertToZUp(mesh, lookupTable["root"]);
		}
	}

	int numMainNodes = lCurrentScene->GetNodeCount();
	for (int i = 0; i < numMainNodes; i++)
	{
		FbxNode* pNode = lCurrentScene->GetNode(i);
		FbxNodeAttribute* Attr = pNode->GetNodeAttribute();
		if (Attr && Attr->GetAttributeType() == FbxNodeAttribute::eSkeleton)
		{
			const char* lpNodeName = pNode->GetName();
			QString sNodeName(lpNodeName);
			if (sNodeName.contains("head"))
			{
				break;
			}
			if (lookupTable.find(sNodeName) != lookupTable.end())
			{
				FbxNode* pPoseNode = lookupTable[sNodeName];
				//pNode->Copy(*pPoseNode);
				pNode->LclRotation.Set(pPoseNode->LclRotation.Get());
				pNode->LclScaling.Set(pPoseNode->LclScaling.Get());
				pNode->LclTranslation.Set(pPoseNode->LclTranslation.Get());
				pNode->PreRotation.Set(pPoseNode->PreRotation.Get());
				pNode->PostRotation.Set(pPoseNode->PostRotation.Get());
			}
		}
	}
	if (pProgress) pProgress->step();

	// close pose scene
	pPoseScene->Destroy();

	return true;
}
#endif

#if USE_OPENFBX
bool FbxTools::loadAndPose(QString poseFilePath, FbxScene* lCurrentScene, DzProgress* pProgress, bool bConvertToZUp)
{
	OpenFBXInterface* openFBX = OpenFBXInterface::GetInterface();

	FbxScene* pPoseScene = openFBX->CreateScene("My Scene");
	if (openFBX->LoadScene(pPoseScene, poseFilePath.toLocal8Bit().data()) == false)
	{
		return false;
	}
	if (pProgress) pProgress->step();

	// make nodename lookup table
	QMap<QString, FbxNode*> lookupTable;
	int numPoseNodes = pPoseScene->GetNodeCount();
	for (int i = 0; i < numPoseNodes; i++)
	{
		FbxNode* pNode = pPoseScene->GetNode(i);
		const char* lpNodeName = pNode->GetName();
		QString sNodeName(lpNodeName);
		lookupTable.insert(sNodeName, pNode);
	}
	if (pProgress) pProgress->step();

	// Convert Pose Scene to Zup
	if (bConvertToZUp)
	{
		FbxMesh* mesh = (FbxMesh*) pPoseScene->GetGeometry(0);
		if (mesh)
		{
			convertToZUp(mesh, lookupTable["root"]);
		}
	}

	int numMainNodes = lCurrentScene->GetNodeCount();
	for (int i = 0; i < numMainNodes; i++)
	{
		FbxNode* pNode = lCurrentScene->GetNode(i);
		FbxNodeAttribute* Attr = pNode->GetNodeAttribute();
		if (Attr && Attr->GetAttributeType() == FbxNodeAttribute::eSkeleton)
		{
			const char* lpNodeName = pNode->GetName();
			QString sNodeName(lpNodeName);
			if (sNodeName == "RootNode")
				continue;
			if (lookupTable.find(sNodeName) != lookupTable.end())
			{
				FbxNode* pPoseNode = lookupTable[sNodeName];
				pNode->Copy(*pPoseNode);
				//pNode->LclRotation.Set(pPoseNode->LclRotation.Get());
				//pNode->LclScaling.Set(pPoseNode->LclScaling.Get());
				//pNode->LclTranslation.Set(pPoseNode->LclTranslation.Get());
				//pNode->PreRotation.Set(pPoseNode->PreRotation.Get());
				//pNode->PostRotation.Set(pPoseNode->PostRotation.Get());
			}
		}
	}
	if (pProgress) pProgress->step();

	// close pose scene
	pPoseScene->Destroy();

	return true;
}
#endif

//bool loadAndPose(QString poseFilePath, FbxScene* lCurrentScene, DzProgress* pProgress )
//{
//	OpenFBXInterface* openFBX = OpenFBXInterface::GetInterface();
//
//	FbxScene* pPoseScene = openFBX->CreateScene("My Scene");
//	if (openFBX->LoadScene(pPoseScene, poseFilePath.toLocal8Bit().data()) == false)
//	{
//		return false;
//	}
//	if (pProgress) pProgress->step();
//
//	// make nodename lookup table
//	QMap<QString, FbxNode*> lookupTable;
//	int numMainNodes = lCurrentScene->GetNodeCount();
//	for (int i = 0; i < numMainNodes; i++)
//	{
//		FbxNode* pNode = lCurrentScene->GetNode(i);
//		const char* lpNodeName = pNode->GetName();
//		QString sNodeName(lpNodeName);
//		lookupTable.insert(sNodeName, pNode);
//	}
//	if (pProgress) pProgress->step();
//
//	FbxPose* bindPose = lCurrentScene->GetPose(0);
//
//	int numPoseNodes = pPoseScene->GetNodeCount();
//	for (int i = 0; i < numPoseNodes; i++)
//	{
//		FbxNode* poseNode = pPoseScene->GetNode(i);
//		const char* lpNodeName = poseNode->GetName();
//		QString sNodeName(lpNodeName);
//		// find node in main scene
//		FbxNode* mainNode = lookupTable[sNodeName];
//		if (mainNode)
//		{
//			mainNode->Copy(*poseNode);
//		}
//	}
//	if (pProgress) pProgress->step();
//
//	// close pose scene
//	pPoseScene->Destroy();
//
//	return true;
//}



int FbxTools::convertToZUp(FbxMesh* mesh, FbxNode* rootNode)
{
	int correction = 0;
	FbxVector4 eulerRotation;
	bool bRotate = false;
	if (bRotate == false)
	{
		// 1. Find Bounding Box
		FbxVector4* result = calculateBoundingVolume(mesh);
		// 2. Check longest axis
		FbxVector4 cloudSize = result[0];
		FbxVector4 cloudCenter = result[1];
		if (cloudSize[1] > cloudSize[2])
		{
			// 3. If longest axis is not Y, then flip
			bRotate = true;
			// check Y value to figure out which direction to flip
			if (cloudCenter[1] > 0)
			{
				// mesh is +Yup
				correction = 90;
				eulerRotation = FbxVector4(correction, 0, 0);
			}
			else
			{
				correction = -90;
				eulerRotation = FbxVector4(correction, 0, 0);
			}
		}
		delete[] result;
	}
	//FbxNode* rootNode = lookupTable[QString("root")];
	if (rootNode && bRotate)
	{
		// HARD-CODED 90-deg X-axis rotation of root node....
		// TODO: detect and apply global axis correction as needed
		rootNode->LclRotation.Set(eulerRotation + rootNode->LclRotation.Get());
	}

	return correction;
}

bool FbxTools::flipAndBakeVertexBuffer(FbxMesh* mesh, FbxNode* rootNode, FbxVector4* vertex_buffer)
{
	if (convertToZUp(mesh, rootNode) == false)
		return false;
	bakePoseToVertexBuffer(vertex_buffer, &getAffineMatrix(NULL, mesh->GetNode()), nullptr, mesh);

	return true;
}

FbxCluster* FbxTools::findClusterFromNode(FbxNode* pNode)
{
	// debug
	int numDstConnections = pNode->GetDstObjectCount();
	int numSrcConnections = pNode->GetSrcObjectCount();
	const char* lpNodeName = pNode->GetName();

	FbxCluster* pCluster1 = (FbxCluster*)pNode->GetSrcObject(FbxCriteria::ObjectType(FbxCluster::ClassId));
	FbxCluster* pCluster2 = (FbxCluster*)pNode->GetDstObject(FbxCriteria::ObjectType(FbxCluster::ClassId));
	FbxCluster* pCluster = nullptr;

	if (pCluster1)
	{
		pCluster = pCluster1;
	}
	else if (pCluster2)
	{
		pCluster = pCluster2;
	}

	return pCluster;

}

bool FbxTools::isBone(FbxNode* pNode, int skeletonTypeMask )
{
	if (!pNode)
		return false;

	FbxNodeAttribute* fbxAttribute = pNode->GetNodeAttribute();
	FbxNodeAttribute::EType eNodeType;
	FbxSkeleton::EType eSkeletonType;
	if (fbxAttribute)
	{
		eNodeType = fbxAttribute->GetAttributeType();
		if (eNodeType == FbxNodeAttribute::EType::eSkeleton)
		{
			const FbxSkeleton* fbxSkeleton = pNode->GetSkeleton();
			eSkeletonType = fbxSkeleton->GetSkeletonType();

			if (skeletonTypeMask == -1)
			{
				return true;
			}
			else
			{
				if (eSkeletonType == skeletonTypeMask)
				{
					return true;
				}
			}
		}
	}

	return false;
}

bool FbxTools::checkIfChildrenAreBones(FbxNode* pNode)
{
	int numChildren = pNode->GetChildCount();
	for (int i = 0; i < numChildren; i++)
	{
		FbxNode* fbxNode = pNode->GetChild(i);
		if (isBone(fbxNode))
			return true;

		checkIfChildrenAreBones(fbxNode);
	}

	return false;
}

void FbxTools::inspectNode(FbxNode* pNode)
{
	if (!pNode)
	{
		printf("pNode is null.\n");
		return;
	}

	const char* lpNodeName = pNode->GetName();
	const char* lpParentName = nullptr;
	FbxNode* pParentNode = pNode->GetParent();
	if (pParentNode)
	{
		lpParentName = pParentNode->GetName();
	}
	int numChildren = pNode->GetChildCount();

	const FbxNull* pNullNode = pNode->GetNull();
	if (pNullNode)
	{
		printf("pNode is FbxNull.\n");
		return;
	}

	FbxNodeAttribute* fbxAttribute = pNode->GetNodeAttribute();
	FbxNodeAttribute::EType eNodeType;
	FbxSkeleton::EType eSkeletonType;
	if (fbxAttribute)
	{
		eNodeType = fbxAttribute->GetAttributeType();
		if (eNodeType == FbxNodeAttribute::EType::eSkeleton)
		{
			const FbxSkeleton* fbxSkeleton = pNode->GetSkeleton();
			eSkeletonType = fbxSkeleton->GetSkeletonType();
		}
	}

	printf("pNode is %s.\n", lpNodeName);

}

FbxNode* FbxTools::findAssociatedSkeletonRoot(FbxMesh* pMesh)
{
	if (pMesh == nullptr)
		return nullptr;

	FbxNode* pSkeletonRoot = nullptr;
	FbxCluster* pCluster = nullptr;

	int numSkins = pMesh->GetDeformerCount(FbxDeformer::EDeformerType::eSkin);
	for (int i = 0; i < numSkins; i++)
	{
		fbxsdk::FbxSkin* pSkin = (FbxSkin*) pMesh->GetDeformer(i, FbxDeformer::EDeformerType::eSkin);
		int numClusters = pSkin->GetClusterCount();
		for (int j = 0; j < numClusters; j++)
		{
			pCluster = pSkin->GetCluster(j);
			if (pCluster)
			{
				break;
			}
		}
	}

	if (pCluster)
	{
		FbxNode* pLinkNode = pCluster->GetLink();
		pSkeletonRoot = pLinkNode;
		if (pLinkNode)
		{
			// find root of bone, go up parent tree until non skeleton parent is found, or skeleton is null
			FbxNode* pParentNode = pLinkNode->GetParent();
			while (pParentNode && isBone(pParentNode))
			{
				pSkeletonRoot = pParentNode;
				if ( pParentNode->GetParent() == nullptr ||
					isBone(pParentNode->GetParent()) == false )
				{
					break;
				}
				pParentNode = pParentNode->GetParent();
			}
		}
	}

	return pSkeletonRoot;
}

bool FbxTools::getBindMatrixFromCluster(FbxNode* pNode, FbxAMatrix &returnMatrix)
{
	bool bSuccess = false;
	if (pNode)
	{
		FbxCluster* pCluster = FbxTools::findClusterFromNode(pNode);
		if (pCluster)
		{
			bSuccess = true;
			FbxAMatrix bindMatrix;
			pCluster->GetTransformLinkMatrix(bindMatrix);
			returnMatrix = bindMatrix;
		}
	}
	return bSuccess;
}

bool FbxTools::getBindTranslationFromCluster(FbxNode* pNode, FbxVector4 &returnTranslation)
{
	bool bSuccess = false;
	if (pNode) 
	{
		FbxAMatrix bindMatrix;
		bSuccess = FbxTools::getBindMatrixFromCluster(pNode, bindMatrix);
		if (bSuccess)
		{
			FbxVector4 bindTranslation(0, 0, 0, 1);
			bindTranslation = bindMatrix.GetT();
			bindTranslation[3] = 1.0f;
			returnTranslation = bindTranslation;
		}
	}
	return bSuccess;
}

bool FbxTools::getBindRotationFromCluster(FbxNode* pNode, FbxVector4& returnRotation)
{
	bool bSuccess = false;
	if (pNode)
	{
		FbxAMatrix bindMatrix;
		bSuccess = FbxTools::getBindMatrixFromCluster(pNode, bindMatrix);
		if (bSuccess)
		{
			FbxVector4 bindRotation(0, 0, 0, 1);
			bindRotation = bindMatrix.GetR();
			bindRotation[3] = 1.0f;
			returnRotation = bindRotation;
		}
	}
	return bSuccess;
}
