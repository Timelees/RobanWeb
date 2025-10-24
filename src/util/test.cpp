// Simple Assimp usage test
#include <iostream>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/version.h>

int main(int argc, char** argv) {
    std::cout << "Assimp runtime version: "
              << aiGetVersionMajor() << "." << aiGetVersionMinor() << "." << aiGetVersionPatch();
    const char *branch = aiGetBranchName();
    if (branch) std::cout << " (branch: " << branch << ")";
    std::cout << std::endl;

    Assimp::Importer importer;

    if (argc < 2) {
        std::cout << "No model file provided. Usage: test <model-file>\n";
        std::cout << "Created Importer object successfully. If you want to fully test reading, pass a model path as argument." << std::endl;
        return 0;
    }

    const char* modelPath = argv[1];
    std::cout << "Attempting to load model: " << modelPath << std::endl;

    const aiScene* scene = importer.ReadFile(modelPath,
        aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes);

    if (!scene) {
        std::cerr << "Failed to load model: " << importer.GetErrorString() << std::endl;
        return 2;
    }

    std::cout << "Model loaded. MeshCount=" << scene->mNumMeshes << " MaterialCount=" << scene->mNumMaterials << std::endl;
    if (scene->mNumMeshes > 0 && scene->mMeshes) {
        std::cout << "First mesh vertices=" << scene->mMeshes[0]->mNumVertices << " faces=" << scene->mMeshes[0]->mNumFaces << std::endl;
    }

    return 0;
}
