//
// Created by Cam on 2024-11-12.
//

#include "SculptProcess.h"

#include "Sculpture.h"
#include "fileIO/MeshHandler.h"

#define my_assert(cond)                             \
    if (!(cond)) {                                  \
        fprintf(stderr, "MCUT error: %s\n", #cond); \
        std::abort();                               \
    }

SculptProcess::SculptProcess(const std::shared_ptr<Mesh>& model)
    : Scene()
    , model(model)
    , m_sculpture(nullptr)
    , m_step(0)
    , context(MC_NULL_HANDLE)
{
    m_sculpture = new Sculpture(model, m_config.materialWidth, m_config.materialHeight);
    prepareBody(m_sculpture, 1);

    prepareBody(new Body(model), 1);

    plan();

    std::cout << "Size test " << sizeof(vec3f) << "\n";

    // Create the context
    McResult status = mcCreateContext(&context, MC_NULL_HANDLE);
    my_assert (status == MC_NO_ERROR);

}

SculptProcess::~SculptProcess()
{
    std::cout << "SP destroy\n";

    // Free context memory
    McResult status = mcReleaseContext(context);
    my_assert (status == MC_NO_ERROR);
}

void SculptProcess::next()
{
    if (m_step >= m_steps.size()) return;

    std::cout << "Step (" << m_step << " / " << m_steps.size() << "): ";

    std::cout << "Origin = " << m_steps[m_step].origin << ", Normal = " << m_steps[m_step].normal << "\n";

    section(m_steps[m_step].normal, m_steps[m_step].origin.dot(m_steps[m_step].normal));

    m_step++;
}

void SculptProcess::plan()
{
    const ConvexHull& hull = m_entities[1].body->hull();

    for (uint32_t i = 0; i < hull.facetCount(); i++) {
        uint32_t idx = hull.faces()[i][0];
        m_steps.push_back({vec3f(hull.vertices()[idx][0], hull.vertices()[idx][1], hull.vertices()[idx][2]), hull.facetNormal(i)});
    }
}

void SculptProcess::section(const vec3f& normal, float offset)
{

    const McDouble norm[] = {0, 1, 1};
    static McDouble sectionOffset = 0.45;
    McEvent dispatchEvent = MC_NULL_HANDLE;

    std::cout << m_sculpture->mesh()->vertexCount() << " " << m_sculpture->mesh()->faceCount() << " woda\n";

    McFloat srcMeshVertices[] = {
            -5, -5, 5,  // vertex 0
            5, -5, 5,   // vertex 1
            5, 5, 5,    // vertex 2
            -5, 5, 5,   // vertex 3
            -5, -5, -5, // vertex 4
            5, -5, -5,  // vertex 5
            5, 5, -5,   // vertex 6
            -5, 5, -5   // vertex 7
    };

    McUint32 srcMeshFaces[] = {
            0, 1, 2, 3, // face 0
            7, 6, 5, 4, // face 1
            1, 5, 6, 2, // face 2
            0, 3, 7, 4, // face 3
            3, 2, 6, 7, // face 4
            4, 5, 1, 0  // face 5
    };

    McUint32 srcMeshFaceSizes[] = { 4, 4, 4, 4, 4, 4};

    McUint32 srcMeshVertexCount = 8;
    McUint32 srcMeshFaceCount = 6;

    McResult status = mcEnqueueDispatchPlanarSection(
            context,
            MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_ENFORCE_GENERAL_POSITION,
            srcMeshVertices,
            srcMeshFaces,
            srcMeshFaceSizes,
            srcMeshVertexCount,
            srcMeshFaceCount,
            norm,
            sectionOffset,
            0,
            nullptr,
            &dispatchEvent
    );
    sectionOffset += 1;

//    const McDouble mcNormal[] = {normal.x, normal.y, normal.z};
//    const McDouble mcNormal[] = {0, 1, 0};


    // Initiate planar section
//    McResult status = mcEnqueueDispatchPlanarSection(
//            context,
//            MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_ENFORCE_GENERAL_POSITION,
//            m_sculpture->mesh()->vertices(),
//            m_sculpture->mesh()->faces().faces(),
//            m_sculpture->mesh()->faces().faceSizes(),
//            m_sculpture->mesh()->vertexCount(),
//            m_sculpture->mesh()->faceCount(),
//            mcNormal,
//            offset,
//            0,
//            nullptr,
//            &dispatchEvent
//    );

    std::cout << sectionOffset << " after dispatch\n";
    my_assert (status == MC_NO_ERROR);

    status =  mcWaitForEvents(1,&dispatchEvent );
    my_assert (status == MC_NO_ERROR);

    process();

}

void SculptProcess::remove()
{
    // Do cut
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
//    my_assert (status == MC_NO_ERROR);

    process();
}

void SculptProcess::process()
{
    // Query the number of available connected components after the cut
    McUint32 connectedComponentCount;
    std::vector<McConnectedComponent> connectedComponents;

    std::vector<std::shared_ptr<Mesh>> fragments;

    McResult status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &connectedComponentCount);
    my_assert (status == MC_NO_ERROR);

    if (connectedComponentCount == 0) {
        std::cout << "no connected components found\n";
//        exit(EXIT_FAILURE);
    }

    connectedComponents.resize(connectedComponentCount);

    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connectedComponents.size(), connectedComponents.data(), NULL);
    my_assert (status == MC_NO_ERROR);


    //  Query the data of each connected component
    for (int i = 0; i < (int)connectedComponents.size(); ++i) {
        McConnectedComponent cc = connectedComponents[i];

        // Component type
        auto ccType = (McConnectedComponentType)0;

        status = mcGetConnectedComponentData(context,cc,MC_CONNECTED_COMPONENT_DATA_TYPE,sizeof(McConnectedComponentType),&ccType,NULL);
        my_assert (status == MC_NO_ERROR);

        if (ccType == MC_CONNECTED_COMPONENT_TYPE_FRAGMENT) {
            auto seal = (McFragmentSealType)0;
//            auto fragmentLocation = (McFragmentLocation)0;
            auto patchLocation = (McPatchLocation)0;

            status = mcGetConnectedComponentData(context,
                                                 cc,
                                                 MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE,
                                                 sizeof(McFragmentSealType),
                                                 &seal,
                                                 NULL);
            my_assert(status == MC_NO_ERROR);

//            status = mcGetConnectedComponentData(context,
//                                                 cc,
//                                                 MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION,
//                                                 sizeof(McFragmentLocation),
//                                                 &fragmentLocation,
//                                                 NULL);
//            my_assert(status == MC_NO_ERROR);

            status = mcGetConnectedComponentData(context,
                                                 cc,
                                                 MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION,
                                                 sizeof(McPatchLocation),
                                                 &patchLocation,
                                                 NULL);
            my_assert(status == MC_NO_ERROR);

            if (seal == MC_FRAGMENT_SEAL_TYPE_COMPLETE && patchLocation == MC_PATCH_LOCATION_INSIDE) {
                fragments.push_back(prepare(cc));
            }

        }
    }

    // Free individual component memory
    status = mcReleaseConnectedComponents(context, 0, NULL);
    my_assert (status == MC_NO_ERROR);

    attachFragments(fragments);
}

std::shared_ptr<Mesh> SculptProcess::prepare(const McConnectedComponent& cc)
{

    // Vertices

    McSize numBytes = 0;
    McResult status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &numBytes);
    my_assert (status == MC_NO_ERROR);

    McUint32 ccVertexCount = (McUint32)(numBytes / (sizeof(McFloat) * 3ull));
    std::vector<McFloat> ccVertices(ccVertexCount * 3u);

    status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, numBytes, (McVoid*)ccVertices.data(), NULL);
    my_assert (status == MC_NO_ERROR);


    // Faces

    numBytes = 0;

    status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);
    my_assert (status == MC_NO_ERROR);

    std::vector<McUint32> ccFaceIndices;
    ccFaceIndices.resize(numBytes / sizeof(McUint32));

    status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, (McVoid*)ccFaceIndices.data(), NULL);
    my_assert (status == MC_NO_ERROR);


    // Face sizes (vertices per face)

    numBytes = 0;

    status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
    my_assert (status == MC_NO_ERROR);

    std::vector<McUint32> ccFaceSizes;
    ccFaceSizes.resize(numBytes / sizeof(McUint32));

    status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, (McVoid*)ccFaceSizes.data(), NULL);
    my_assert (status == MC_NO_ERROR);

    std::cout << ccVertexCount << " " << ccFaceSizes.size() << " Next fragment\n";

    return std::make_shared<Mesh>(
                ccVertices.data(), ccVertexCount,
                ccFaceIndices.data(), ccFaceSizes.data(), ccFaceSizes.size()
                );
}

void SculptProcess::attachFragments(const std::vector<std::shared_ptr<Mesh>>& fragments)
{
    uint32_t sculptureIdx = identifySculpture(fragments);
    m_sculpture->overrwrite(fragments[sculptureIdx]);

    std::cout << fragments.size() << " " << sculptureIdx << " AF\n";
    if (!m_entities.empty()) m_entities[0].render->generate(); // Update render, assumes sculpture is the first entity

    for (uint32_t i = 0; i < fragments.size(); i++) {
        MeshHandler::exportMesh(fragments[i], "..\\out\\frag" + std::to_string(i) + ".obj");
//        if (i == sculptureIdx) continue;
//
//        prepareBody(new Body(&m_physicsCommon, m_world, fragments[i]));
////        prepareBody(new Body(fragments[i]));
    }
}

uint32_t SculptProcess::identifySculpture(const std::vector<std::shared_ptr<Mesh>>& fragments)
{
    uint32_t idx = 0, count = 0;

    for (uint32_t i = 0; i < fragments.size(); i++) {
        if (fragments[i]->faceCount() > count) {
            count = fragments[i]->faceCount();
            idx = i;
        }
    }

    return idx;
}