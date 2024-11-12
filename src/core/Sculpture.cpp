//
// Created by Cam on 2024-11-10.
//

#include "Sculpture.h"

#include "geometry/MeshBuilder.h"

#include "mcut/mcut.h"

Sculpture::Sculpture(const std::shared_ptr<Mesh> &model, float width, float height)
    : Body(MeshBuilder::box(width, width, height))
    , model(model)
    , m_width(width)
    , m_height(height)
{

    scaleToFit(m_width, m_height);

    m_mesh->scale(1, m_height / height, 1);
    m_mesh->translate(0, m_height / 2, 0);

    std::cout << "Volume ratio: " << 100 * bulkUsageRatio() << "% material usage, " << 100 * remainderRatio() << "% Remaining\n";


//    McFloat srcMeshVertices[] = {
//            -5, -5, 5,  // vertex 0
//            5, -5, 5,   // vertex 1
//            5, 5, 5,    // vertex 2
//            -5, 5, 5,   // vertex 3
//            -5, -5, -5, // vertex 4
//            5, -5, -5,  // vertex 5
//            5, 5, -5,   // vertex 6
//            -5, 5, -5   // vertex 7
//    };
//
//    McUint32 srcMeshFaces[] = {
//            0, 1, 2, 3, // face 0
//            7, 6, 5, 4, // face 1
//            1, 5, 6, 2, // face 2
//            0, 3, 7, 4, // face 3
//            3, 2, 6, 7, // face 4
//            4, 5, 1, 0  // face 5
//    };
//
//    McUint32 srcMeshFaceSizes[] = { 4, 4, 4, 4, 4, 4};
//
//    McUint32 srcMeshVertexCount = 8;
//    McUint32 srcMeshFaceCount = 6;
//
//    // the cut mesh (a quad formed of two triangles)
//
//    McFloat cutMeshVertices[] = {
//            -20, -4, 0, // vertex 0
//            0, 20, 20,  // vertex 1
//            20, -4, 0,  // vertex 2
//            0, 20, -20  // vertex 3
//    };
//
//    McUint32 cutMeshFaces[] = {
//            0, 1, 2, // face 0
//            0, 2, 3  // face 1
//    };
//
//    // McUint32 cutMeshFaceSizes[] = { 3, 3};
//
//    McUint32 cutMeshVertexCount = 4;
//    McUint32 cutMeshFaceCount = 2;
//
////    m_sculpture = std::make_shared<Mesh>(
////            srcMeshVertices, srcMeshVertexCount,
////            srcMeshFaces, srcMeshFaceSizes, srcMeshFaceCount
////    );
//
//    // Create the context
//    McContext context = MC_NULL_HANDLE;
//    McResult status = mcCreateContext(&context, MC_NULL_HANDLE);
////    my_assert (status == MC_NO_ERROR);
//
//    // Do cut
//    status = mcDispatch(
//            context,
//            MC_DISPATCH_VERTEX_ARRAY_FLOAT,
//            srcMeshVertices,
//            srcMeshFaces,
//            srcMeshFaceSizes,
//            srcMeshVertexCount,
//            srcMeshFaceCount,
//            cutMeshVertices,
//            cutMeshFaces,
//            nullptr, // cutMeshFaceSizes, // no need to give 'cutMeshFaceSizes' parameter since the cut-mesh is a triangle mesh
//            cutMeshVertexCount,
//            cutMeshFaceCount);
////    my_assert (status == MC_NO_ERROR);
//
//    // Query the number of available connected components after the cut
//    McUint32 connectedComponentCount;
//    std::vector<McConnectedComponent> connectedComponents;
//
//    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &connectedComponentCount);
////    my_assert (status == MC_NO_ERROR);
//
//    if (connectedComponentCount == 0) {
//        fprintf(stdout, "no connected components found\n");
//        exit(EXIT_FAILURE);
//    }
//
//    connectedComponents.resize(connectedComponentCount); // allocate for the amount we want to get
//
//    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connectedComponents.size(), connectedComponents.data(), NULL);
////    my_assert (status == MC_NO_ERROR);
//
//    std::vector<std::shared_ptr<Mesh>> fragments;
//
//    //  Query the data of each connected component
//    for (McInt32 i = 0; i < (McInt32)connectedComponents.size(); ++i) {
//        McConnectedComponent cc = connectedComponents[i];
//        McSize numBytes = 0;
//
//        // Vertices
//
//        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &numBytes);
////        my_assert (status == MC_NO_ERROR);
//
//        McUint32 ccVertexCount = (McUint32)(numBytes / (sizeof(McFloat) * 3ull));
//        std::vector<McFloat> ccVertices(ccVertexCount * 3u);
//
//        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, numBytes, (McVoid*)ccVertices.data(), NULL);
////        my_assert (status == MC_NO_ERROR);
//
//        // Faces
//
//        numBytes = 0;
//
//        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);
////        my_assert (status == MC_NO_ERROR);
//
//        std::vector<McUint32> ccFaceIndices;
//        ccFaceIndices.resize(numBytes / sizeof(McUint32));
//
//        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, (McVoid*)ccFaceIndices.data(), NULL);
////        my_assert (status == MC_NO_ERROR);
//
//        // Face sizes (vertices per face)
//
//        numBytes = 0;
//
//        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
////        my_assert (status == MC_NO_ERROR);
//
//        std::vector<McUint32> ccFaceSizes;
//        ccFaceSizes.resize(numBytes / sizeof(McUint32));
//
//        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, (McVoid*)ccFaceSizes.data(), NULL);
////        my_assert (status == MC_NO_ERROR);
//
//        std::cout << ccVertexCount << " " << ccFaceSizes.size() << " Next fragment\n";
////        if (ccVertexCount == 10 && ccFaceSizes.size() == 5) continue;
//        if (i != 11) continue;
//
//        fragments.push_back(std::make_shared<Mesh>(
//                ccVertices.data(), ccVertexCount,
//                ccFaceIndices.data(), ccFaceSizes.data(), ccFaceSizes.size()
//        ));
//
//        // Convert result into a mesh
////        m_sculpture =
////        ccVertices.data();
////        ccVertexCount;
////
////        ccFaceIndices;
////        ccFaceSizes;
////        ccFaceSizes.size();
//
//    }
//
//    std::cout << fragments.size() << " all built\n";
//
//
//    uint32_t idx = 0, max = fragments[0]->faceCount();
//    for (uint32_t i = 0; i < fragments.size(); i++) {
//        std::cout << idx << " " << i << " " << max << "\n";
//        if(fragments[i]->faceCount() > max) {
//            max = fragments[i]->faceCount();
//            idx = i;
//        }
//    }
//
//    m_sculpture = fragments[idx];
//
//    // Free individual component memory
//    status = mcReleaseConnectedComponents(context, 0, NULL);
////    my_assert (status == MC_NO_ERROR);
//
//    // Free context memory
//    status = mcReleaseContext(context);
//    my_assert (status == MC_NO_ERROR);

//    m_fragments.push_back(new Body(m_sculpture));
}

//void Sculpture::setRenderer(Qt3DCore::QEntity *parent, Qt3DExtras::Qt3DWindow *view)
//{
//    m_render = new RenderEntity(parent, view);
//    m_render->add(m_mesh);
//
//    auto material = new Qt3DExtras::QDiffuseSpecularMaterial();
//    material->setShininess(0);
//    material->setDiffuse(QColor::fromRgbF(0.2f, 0.2f, 0.2f, 1));
//    material->setAmbient(QColor::fromRgbF(0, 0, 0.8f, 1));
//
//    m_render->add(m_sculpture, material);
//
////    if (m_hullOK) m_render->add(std::make_shared<Mesh>(m_hull));
//
//    updateRenderer();
//}

void Sculpture::scaleToFit(float width, float maxHeight)
{
    // Find the maximum dimensions of the mesh
    float xNear, xFar, yNear, yFar, zNear, zFar;
    model->xExtent(xNear, xFar);
    model->yExtent(yNear, yFar);
    model->zExtent(zNear, zFar);

    model->translate(-(xNear + xFar) / 2, -yNear, -(zNear + zFar) / 2); // Center the model in the center
    model->rotate(0, 1, 0, atanf((xFar - xNear) / (zFar - zNear))); // Rotate the model to make best use of diagonal space

    // Re-evaluate extents to figure out scaling requirements
    model->xExtent(xNear, xFar);
    model->zExtent(zNear, zFar);

    model->translate(-(xNear + xFar) / 2, 0, -(zNear + zFar) / 2); // Adjust model in the center

    m_height = yFar - yNear;

    // Calculate the scale factor to make the mesh as large as possible, within the specified limits
    float scalar = std::min(width / (xFar - xNear), width / (zFar - zNear));
    if (m_height * scalar > maxHeight) {
        scalar = maxHeight / m_height;
    }

    m_height *= scalar;
//    m_height = maxHeight;

    model->scale(scalar); // Scale model to fit within specified limits
}

void Sculpture::update()
{
//    m_mesh->rotate(0, 1, 0, 0.001f);

}

const std::shared_ptr<Mesh>& Sculpture::sculpture()
{
    return m_mesh;
}


float Sculpture::width() const
{
    return m_width;
}
float Sculpture::height() const
{
    return m_height;
}

float Sculpture::initialVolume() const
{
    return m_width * m_width * m_height;
}
float Sculpture::currentVolume() const
{
    return m_mesh->volume();
}
float Sculpture::finalVolume() const
{
    return model->volume();
}

float Sculpture::bulkUsageRatio() const
{
    return finalVolume() / initialVolume();
}
float Sculpture::remainderRatio() const
{
    float fVolume = finalVolume();

    return (currentVolume() - fVolume) / (initialVolume() - fVolume);
}