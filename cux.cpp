//
// Created by Shane Scott on 8/21/20.
//

#include <vtkGLTFExporter.h>

#include <vtkSmartPointer.h>

#include <vtkArrowSource.h>
#include <vtkGeometryFilter.h>
#include <vtkSphereSource.h>
#include <vtkMath.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkNamedColors.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkXMLUnstructuredGridReader.h>
#include <vtkUnstructuredGrid.h>

#include "Eigen/Dense"

#include <array>

int vtk_example(){
    vtkSmartPointer<vtkNamedColors> colors =
            vtkSmartPointer<vtkNamedColors>::New();

    // Set the background color.
    vtkColor3d backgroundColor = colors->GetColor3d("SlateGray");

    //Create an arrow.
    vtkSmartPointer<vtkArrowSource> arrowSource =
            vtkSmartPointer<vtkArrowSource>::New();

    // Generate a random start and end point
    double startPoint[3];
    double endPoint[3];
    vtkSmartPointer<vtkMinimalStandardRandomSequence> rng =
            vtkSmartPointer<vtkMinimalStandardRandomSequence>::New();
    rng->SetSeed(8775070); // For testing.
    for (auto i = 0; i < 3; ++i)
    {
        rng->Next();
        startPoint[i] = rng->GetRangeValue(-10, 10);
        rng->Next();
        endPoint[i] = rng->GetRangeValue(-10, 10);
    }

    // Compute a basis
    double normalizedX[3];
    double normalizedY[3];
    double normalizedZ[3];

    // The X axis is a vector from start to end
    vtkMath::Subtract(endPoint, startPoint, normalizedX);
    double length = vtkMath::Norm(normalizedX);
    vtkMath::Normalize(normalizedX);

    // The Z axis is an arbitrary vector cross X
    double arbitrary[3];
    for (auto i = 0; i < 3; ++i)
    {
        rng->Next();
        arbitrary[i] = rng->GetRangeValue(-10, 10);
    }
    vtkMath::Cross(normalizedX, arbitrary, normalizedZ);
    vtkMath::Normalize(normalizedZ);

    // The Y axis is Z cross X
    vtkMath::Cross(normalizedZ, normalizedX, normalizedY);
    vtkSmartPointer<vtkMatrix4x4> matrix =
            vtkSmartPointer<vtkMatrix4x4>::New();

    // Create the direction cosine matrix
    matrix->Identity();
    for (auto i = 0; i < 3; i++)
    {
        matrix->SetElement(i, 0, normalizedX[i]);
        matrix->SetElement(i, 1, normalizedY[i]);
        matrix->SetElement(i, 2, normalizedZ[i]);
    }

    // Apply the transforms
    vtkSmartPointer<vtkTransform> transform =
            vtkSmartPointer<vtkTransform>::New();
    transform->Translate(startPoint);
    transform->Concatenate(matrix);
    transform->Scale(length, length, length);

    // Transform the polydata
    vtkSmartPointer<vtkTransformPolyDataFilter> transformPD =
            vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformPD->SetTransform(transform);
    transformPD->SetInputConnection(arrowSource->GetOutputPort());

    //Create a mapper and actor for the arrow
    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkActor> actor =
            vtkSmartPointer<vtkActor>::New();
    mapper->SetInputConnection(transformPD->GetOutputPort());
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(colors->GetColor3d("Cyan").GetData());

    // Create spheres for start and end point
    vtkSmartPointer<vtkSphereSource> sphereStartSource =
            vtkSmartPointer<vtkSphereSource>::New();
    sphereStartSource->SetCenter(startPoint);
    sphereStartSource->SetRadius(0.8);
    vtkSmartPointer<vtkPolyDataMapper> sphereStartMapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereStartMapper->SetInputConnection(sphereStartSource->GetOutputPort());
    vtkSmartPointer<vtkActor> sphereStart =
            vtkSmartPointer<vtkActor>::New();
    sphereStart->SetMapper(sphereStartMapper);
    sphereStart->GetProperty()->SetColor(colors->GetColor3d("Yellow").GetData());

    vtkSmartPointer<vtkSphereSource> sphereEndSource =
            vtkSmartPointer<vtkSphereSource>::New();
    sphereEndSource->SetCenter(endPoint);
    sphereEndSource->SetRadius(0.8);
    vtkSmartPointer<vtkPolyDataMapper> sphereEndMapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereEndMapper->SetInputConnection(sphereEndSource->GetOutputPort());
    vtkSmartPointer<vtkActor> sphereEnd =
            vtkSmartPointer<vtkActor>::New();
    sphereEnd->SetMapper(sphereEndMapper);
    sphereEnd->GetProperty()->SetColor(colors->GetColor3d("Magenta").GetData());

    //Create a renderer, render window, and interactor
    vtkSmartPointer<vtkRenderer> renderer =
            vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
            vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(640, 480);
    renderWindow->SetWindowName("Oriented Arrow");

    auto style =
            vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
    renderWindowInteractor->SetInteractorStyle(style);

    //Add the actor to the scene
    renderer->AddActor(actor);
    renderer->AddActor(sphereStart);
    renderer->AddActor(sphereEnd);
    renderer->SetBackground(backgroundColor.GetData());

    //Render and interact
    renderWindow->Render();
    renderer->GetActiveCamera()->Azimuth(30);
    renderer->GetActiveCamera()->Elevation(30);
    renderer->GetActiveCamera()->Roll(30);
    renderer->ResetCameraClippingRange();

    renderWindowInteractor->Start();

    auto writer =
            vtkSmartPointer<vtkGLTFExporter>::New();
    writer->SetFileName("/Users/sscott/Desktop/cux/crackle.gltf");
    writer->InlineDataOn();
    writer->SetRenderWindow(renderWindow);
    writer->Write();

    return 0;
}


int cuby(){
    std::string cube_path = "/Users/sscott/Desktop/cux/27cubes.vtu";
    auto reader = vtkSmartPointer<vtkXMLUnstructuredGridReader>::New();
    reader->SetFileName(cube_path.c_str());
    reader->Update();
    auto ugrid = reader->GetOutput();

    //Create a renderer, render window, and interactor
    vtkSmartPointer<vtkRenderer> renderer =
            vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
            vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(640, 480);
    renderWindow->SetWindowName("Oriented Arrow");

    auto goer = vtkSmartPointer<vtkGeometryFilter>::New();
    goer->SetInputData(ugrid);
    goer->Update();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(goer->GetOutputPort());
    mapper->Update();
    vtkSmartPointer<vtkActor> actor =
            vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    //Add the actor to the scene
    renderer->AddActor(actor);

    //Render and interact
    renderWindow->Render();
    renderer->GetActiveCamera()->Azimuth(30);
    renderer->GetActiveCamera()->Elevation(30);
    renderer->GetActiveCamera()->Roll(30);
    renderer->ResetCameraClippingRange();


    auto writer =
            vtkSmartPointer<vtkGLTFExporter>::New();
    writer->SetFileName("/Users/sscott/Desktop/cux/27_cube.gltf");
    writer->InlineDataOn();
    writer->SetRenderWindow(renderWindow);
    writer->Write();

    return 0;
}

Eigen::MatrixXd getIntEvalMat(const int n){
    Eigen::MatrixXd m(n,n);
    for (auto foo=0; foo<n; foo++){
        double prod = 1;
        for (auto bar=0; bar<n; bar++){
            m(foo,bar) = prod;
            prod *= foo+1;
        }
    }
    return m;
};

Eigen::VectorXd getPolyApprox(Eigen::VectorXd sequence){
    const int nn = static_cast<int>(sequence.size());
    Eigen::MatrixXd pim = getIntEvalMat(nn);
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dec(pim);
    Eigen::VectorXd coeffs = dec.solve(sequence);
    return coeffs;
}

double evaluatePoly(Eigen::VectorXd coeffs, double x){
    double summer = 0;
    for (auto foo = coeffs.size()-1; foo>=0; foo--){
        summer *= x;
        summer += coeffs(foo);
    }
    return summer;
}

double sumOfIncorrectTerms(Eigen::VectorXd sequence){
    double sum = 0;
    int seqsize = sequence.size();
    for (int foo=1; foo<seqsize; foo++){
        Eigen::VectorXd subseq = sequence.block(0,0,foo,1);
        std::cout << "Subsequence: " << std::endl << subseq << std::endl;
        Eigen::VectorXd papprox = getPolyApprox(subseq);
        std::cout << "Approx: " << std::endl << papprox << std::endl;
        double err = evaluatePoly(papprox, foo+1);
        std::cout << "Err: " <<  err << std::endl;
        std::cout << std::endl;
        sum += err;
        std::cout << "sum: " << static_cast<long long int>(sum) << std::endl;
    }
    return sum;
}

Eigen::VectorXd Zevaluatepoly(Eigen::VectorXd x){
    Eigen::VectorXd evals(x.size());
    for(auto foo=0; foo<x.size(); foo++){
        evals(foo) = evaluatePoly(x, foo+1);
    }
    return evals;
}

void example_leonid(){
//    Eigen::VectorXd fullpoly(4);
//    fullpoly << 0, 0, 0, 1;
    Eigen::VectorXd fullpoly(11);
    fullpoly << 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1;
    Eigen::VectorXd evals = Zevaluatepoly(fullpoly);
    std::cout << "ZZ evaluation is" << std::endl << evals << std::endl;
    double sumtot = sumOfIncorrectTerms(evals);
    std::cout << "Total sum is " << sumtot << std::endl;
}

int main() {
    example_leonid();
    return 0;
//    vtk_example();
}