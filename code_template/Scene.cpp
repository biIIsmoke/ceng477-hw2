#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

#include "tinyxml2.h"
#include "Triangle.h"
#include "Helpers.h"
#include "Scene.h"

using namespace tinyxml2;
using namespace std;


/* my helper functions */

void perspectiveDivision(Vec4 & vertex)
{
	if(vertex.t != 1.0)
	{
		vertex.x=vertex.x/vertex.t;
		vertex.y=vertex.y/vertex.t;
		vertex.z=vertex.z/vertex.t;
		vertex.t=1.0;
	}
}

bool isCulled(Vec4 & projected0, Vec4 & projected1, Vec4 & projected2)
{
	Vec3 _vertex0 = Vec3(projected0.x, projected0.y, projected0.z);
	Vec3 _vertex1 = Vec3(projected1.x, projected1.y, projected1.z);
	Vec3 _vertex2 = Vec3(projected2.x, projected2.y, projected2.z);
	
	Vec3 _line01 = subtractVec3(_vertex0, _vertex1);
	Vec3 _line02 = subtractVec3(_vertex0, _vertex2);

	Vec3 _normalVector = normalizeVec3(crossProductVec3(_line01,_line02));
	
	double product = dotProductVec3(_normalVector, _vertex0);
	if(product > 0) // n.v > 0 so back facing triangle, gets culled
	{
		return true;
	}
	else // n.v < 0 so front facin triangle, doesn't get culled
	{
		return false;
	}
}

bool isVisible(double den, double num, double & t_E, double & t_L)
{
	if(den > 0) //potentially entering
	{
		double _t = num/den;
		if(_t > t_L)
		{
			return false;
		}
		if(_t > t_E)
		{
			t_E = _t;
		}
	}
	else if(den < 0) //potentially leaving
	{
		double _t = num/den;
		if(_t < t_E)
		{
			return false;
		}
		if(_t < t_L)
		{
			t_L = _t;
		}
	}
	else if(num > 0) //line parallel to edge
	{
		return false;
	}
	return true;
}

Color color1MinusColor0(Color color0, Color color1)
{
	return Color(color1.r - color0.r, color1.g - color0.g, color1.b - color0.b);
}

Color colorMultiplier(Color color, double m)
{
	return Color(color.r * m, color.g * m, color.b * m);
}

Color roundColor(Color c)
{
	return Color(round(c.r), round(c.g), round(c.b));
}

void draw(vector<vector<Color>> & image, int x, int y, Color c)
{
	image[x][y]=c;
	//cout << "image color at " << x << "," << y << " is: " << c << endl;
}

void midpoint(vector<vector<Color>> & image, Vec4 & viewported0, Vec4 & viewported1, Color & color0, Color & color1)
{
	if(viewported0.x > viewported1.x) // if vertex1 is left of vertex0, swap them
	{
		midpoint(image, viewported1, viewported0, color1, color0);
		//cout << "called swap" << endl;
		return;
	}
	
	int y = viewported0.y;
	double d = (viewported0.y - viewported1.y) + 0.5 * (viewported1.x - viewported0.x);
	Color c = Color(color0);
	Color dc = colorMultiplier(color1MinusColor0(color0,color1),1/(viewported1.x - viewported0.x));
	
	for(int x = viewported0.x; x <= viewported1.x; ++x)
	{
		draw(image, x, y, roundColor(c));
		if(d<0) // choose NE
		{
			y = y + 1;
			d = d + (viewported0.y - viewported1.y) + (viewported1.x - viewported0.x);
		}
		else // choose E
		{
			d = d + (viewported0.y - viewported1.y);
		}
		c = color1MinusColor0(colorMultiplier(c, -1.0),dc);
	}
}

bool liangBarsky(Vec4 & vertex0, Vec4 & vertex1, Color & color0, Color & color1)
{
	//cout << "liang barsky" << endl;
	double t_E = 0;
	double t_L = 1;
	
	double x_min = -1.0;
	double x_max = 1.0;
	double y_min = -1.0;
	double y_max = 1.0;
	double z_min = -1.0;
	double z_max = 1.0;

	double d_x = vertex1.x-vertex0.x;
	double d_y = vertex1.y-vertex0.y;
	double d_z = vertex1.z-vertex0.z;
	Color d_c = color1MinusColor0(color0,color1);
	
	bool visibility = false;
	if(isVisible(d_x, x_min-vertex0.x, t_E, t_L)) //left
	{
		//cout << "liang barsky 1" << endl;
		if(isVisible(-d_x, vertex0.x-x_max, t_E, t_L)) //right
		{
			//cout << "liang barsky 2" << endl;
			if(isVisible(d_y, y_min-vertex0.y, t_E, t_L)) //bottom
			{
				//cout << "liang barsky 3" << endl;
				if(isVisible(-d_y, vertex0.y-y_max, t_E, t_L)) //top
				{
					//cout << "liang barsky 4" << endl;
					if(isVisible(d_z, z_min-vertex0.z, t_E, t_L)) //front
					{
						//cout << "liang barsky 5" << endl;
						if(isVisible(-d_z, vertex0.z-z_max, t_E, t_L)) //back
						{
							//cout << "liang barsky 6" << endl;
							visibility = true;
							if(t_L < 1)
							{
								vertex1.x = vertex0.x + d_x*t_L;
								vertex1.y = vertex0.y + d_y*t_L;
								vertex1.z = vertex0.z + d_z*t_L;
								color1 = color1MinusColor0(colorMultiplier(color0,-1.0), colorMultiplier(color0,t_L));
							}
							if(t_E>0)
							{
								vertex0.x = vertex0.x + d_x*t_E;
								vertex0.y = vertex0.y + d_y*t_E;
								vertex0.z = vertex0.z + d_z*t_E;
								color0 = color1MinusColor0(colorMultiplier(color0,-1.0), colorMultiplier(color0,t_E));
							}
						}
					}
				}
			}
		}
	}
	return visibility;
}

Matrix4 calculateCameraTransformationMatrix(Camera *camera)
{
	double _cameraRotationArray[4][4] =
	{
		{camera->u.x,camera->u.y,camera->u.z,0},
		{camera->v.x,camera->v.y,camera->v.z,0},
		{camera->w.x,camera->w.y,camera->w.z,0},
		{0,0,0,1}
	};
	double _cameraTranslationArray[4][4] =
	{
		{1,0,0,-(camera->position.x)},
		{0,1,0,-(camera->position.y)},
		{0,0,1,-(camera->position.z)},
		{0,0,0,1}
	};
	cout << "camera transformation done" << endl;
	return multiplyMatrixWithMatrix(Matrix4(_cameraRotationArray),Matrix4(_cameraTranslationArray));
}

Matrix4 calculateCameraProjectionMatrix(Camera *camera)
{
	if(camera->projectionType == 0) //orthographic
	{
		double _cameraOrthographicProjectionArray[4][4] = //calculate orthographic
		{
			{2/(camera->right-camera->left),0,0,-((camera->right+camera->left)/(camera->right-camera->left))},
			{0,2/(camera->top-camera->bottom),0,-((camera->top+camera->bottom)/(camera->top-camera->bottom))},
			{0,0,2/(camera->near-camera->far),-((camera->far+camera->near)/(camera->far-camera->near))},
			{0,0,0,1}
		};
		cout << "camera orthographic projection done" << endl;
		return Matrix4(_cameraOrthographicProjectionArray);
	}
	else //perspective
	{
		//calculate perspective array
		double _cameraPerspectiveArray[4][4] =
		{
			{(2 * camera->near)/(camera->right - camera->left),0,(camera->right + camera->left)/(camera->right - camera->left),0},
			{0,(2 * camera->near)/(camera->top - camera->bottom),(camera->top + camera->bottom)/(camera->top - camera->bottom),0},
			{0,0,-(camera->far + camera->near)/(camera->far - camera->near),-(2 * camera->far * camera->near)/(camera->far - camera->near)},
			{0,0,-1,0}
		};
		
		cout << "camera perspective projection done" << endl;
		return Matrix4(_cameraPerspectiveArray);
	}
}

Matrix4 calculateCameraViewportTransformationMatrix(Camera *camera)
{
	// TODO: check if integer division creates issues
	double _cameraViewportTransformationArray[4][4] =
	{
		{(camera->horRes)/2.0,0,0,(camera->horRes-1)/2.0},
		{0,(camera->verRes)/2.0,0,(camera->verRes-1)/2.0},
		{0,0,0.5,0.5},
		{0,0,0,1}
	};
		
	cout << "camera viewport done" << endl;
	return Matrix4(_cameraViewportTransformationArray);
}

Vec3 findVFromU(Vec3 u)
{
	double _minimumComponent = fmin(fmin(abs(u.x),abs(u.y)),abs(u.z));
	if(abs(u.x) == _minimumComponent) //x is minimum
	{
		return Vec3(0,-u.z,u.y);
	}
	else if(abs(u.y) == _minimumComponent) //y is minimum
	{
		return Vec3(-u.z,0,u.x);
	}
	else if(abs(u.z) == _minimumComponent) // z is minimum
	{
		return Vec3(-u.y,u.x,0);
	}
	else
	{
		cout << "Error: can't find v in findVFromU" << endl;
		return Vec3(0,0,0);
	}
	
}

Matrix4 calculateModelTransformationMatrix(Mesh & mesh, std::vector<Scaling *> & scalings, std::vector<Rotation *> & rotations, std::vector<Translation*> & translations)
{
	Matrix4 _modelTransformationMatrix = getIdentityMatrix();
	
	for(int i=0; i < mesh.numberOfTransformations; i++) //for each transformation
	{
		if(mesh.transformationTypes[i] == 's')
		{
			Scaling * _currentScaling = scalings[mesh.transformationIds[i]-1];
			double _modelScalingArray[4][4] =
			{
				{_currentScaling->sx,0,0,0},
				{0,_currentScaling->sy,0,0},
				{0,0,_currentScaling->sz,0},
				{0,0,0,1}
			};
			_modelTransformationMatrix = multiplyMatrixWithMatrix(Matrix4(_modelScalingArray),_modelTransformationMatrix);
		}
		else if(mesh.transformationTypes[i] == 'r')
		{
			Rotation * _currentRotation = rotations[mesh.transformationIds[i]-1];
			Vec3 u = Vec3(_currentRotation->ux,_currentRotation->uy,_currentRotation->uz);
			Vec3 v = findVFromU(u);
			Vec3 w = crossProductVec3(u,v);
			v = normalizeVec3(v);
			w = normalizeVec3(w);
			Matrix4 _currentRotationMatrix = getIdentityMatrix();
			double _transformArray[4][4] =
			{
				{u.x,u.y,u.z,0},
				{v.x,v.y,v.z,0},
				{w.x,w.y,w.z,0},
				{0,0,0,1}
			};
			_currentRotationMatrix = multiplyMatrixWithMatrix(Matrix4(_transformArray),_currentRotationMatrix);
			double _xRotationArray[4][4] =
			{
				{1,0,0,0},
				{0,cos(_currentRotation->angle*M_PI/180),-sin(_currentRotation->angle*M_PI/180),0},
				{0,sin(_currentRotation->angle*M_PI/180),cos(_currentRotation->angle*M_PI/180),0},
				{0,0,0,1}
			};
			_currentRotationMatrix = multiplyMatrixWithMatrix(Matrix4(_xRotationArray),_currentRotationMatrix);
			double _inverseTransformArray[4][4] =
			{
				{u.x,v.x,w.x,0},
				{u.y,v.y,w.y,0},
				{u.z,v.z,w.z,0},
				{0,0,0,1}
			};
			_currentRotationMatrix = multiplyMatrixWithMatrix(Matrix4(_inverseTransformArray),_currentRotationMatrix);
			
			_modelTransformationMatrix = multiplyMatrixWithMatrix(_currentRotationMatrix,_modelTransformationMatrix);
		}
		else if(mesh.transformationTypes[i] == 't')
		{
			Translation * _currentTranslation = translations[mesh.transformationIds[i]-1];
			double _modelTranslationArray[4][4] =
			{
				{1,0,0,_currentTranslation->tx},
				{0,1,0,_currentTranslation->ty},
				{0,0,1,_currentTranslation->tz},
				{0,0,0,1}
			};
			_modelTransformationMatrix = multiplyMatrixWithMatrix(Matrix4(_modelTranslationArray),_modelTransformationMatrix);
		}
	}
	cout << "model transformation done" << endl;
	return _modelTransformationMatrix;
}

/*
	Parses XML file
*/
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLElement *xmlElement;

	xmlDoc.LoadFile(xmlPath);

	XMLNode *rootNode = xmlDoc.FirstChild();

	// read background color
	xmlElement = rootNode->FirstChildElement("BackgroundColor");
	str = xmlElement->GetText();
	sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	// read culling
	xmlElement = rootNode->FirstChildElement("Culling");
	if (xmlElement != NULL)
	{
		str = xmlElement->GetText();

		if (strcmp(str, "enabled") == 0)
		{
			this->cullingEnabled = true;
		}
		else
		{
			this->cullingEnabled = false;
		}
	}

	// read cameras
	xmlElement = rootNode->FirstChildElement("Cameras");
	XMLElement *camElement = xmlElement->FirstChildElement("Camera");
	XMLElement *camFieldElement;
	while (camElement != NULL)
	{
		Camera *camera = new Camera();

		camElement->QueryIntAttribute("id", &camera->cameraId);

		// read projection type
		str = camElement->Attribute("type");

		if (strcmp(str, "orthographic") == 0)
		{
			camera->projectionType = ORTOGRAPHIC_PROJECTION;
		}
		else
		{
			camera->projectionType = PERSPECTIVE_PROJECTION;
		}

		camFieldElement = camElement->FirstChildElement("Position");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf", &camera->position.x, &camera->position.y, &camera->position.z);

		camFieldElement = camElement->FirstChildElement("Gaze");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf", &camera->gaze.x, &camera->gaze.y, &camera->gaze.z);

		camFieldElement = camElement->FirstChildElement("Up");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf", &camera->v.x, &camera->v.y, &camera->v.z);

		camera->gaze = normalizeVec3(camera->gaze);
		camera->u = crossProductVec3(camera->gaze, camera->v);
		camera->u = normalizeVec3(camera->u);

		camera->w = inverseVec3(camera->gaze);
		camera->v = crossProductVec3(camera->u, camera->gaze);
		camera->v = normalizeVec3(camera->v);

		camFieldElement = camElement->FirstChildElement("ImagePlane");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
			   &camera->left, &camera->right, &camera->bottom, &camera->top,
			   &camera->near, &camera->far, &camera->horRes, &camera->verRes);

		camFieldElement = camElement->FirstChildElement("OutputName");
		str = camFieldElement->GetText();
		camera->outputFilename = string(str);

		this->cameras.push_back(camera);

		camElement = camElement->NextSiblingElement("Camera");
	}

	// read vertices
	xmlElement = rootNode->FirstChildElement("Vertices");
	XMLElement *vertexElement = xmlElement->FirstChildElement("Vertex");
	int vertexId = 1;

	while (vertexElement != NULL)
	{
		Vec3 *vertex = new Vec3();
		Color *color = new Color();

		vertex->colorId = vertexId;

		str = vertexElement->Attribute("position");
		sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

		str = vertexElement->Attribute("color");
		sscanf(str, "%lf %lf %lf", &color->r, &color->g, &color->b);

		this->vertices.push_back(vertex);
		this->colorsOfVertices.push_back(color);

		vertexElement = vertexElement->NextSiblingElement("Vertex");

		vertexId++;
	}

	// read translations
	xmlElement = rootNode->FirstChildElement("Translations");
	XMLElement *translationElement = xmlElement->FirstChildElement("Translation");
	while (translationElement != NULL)
	{
		Translation *translation = new Translation();

		translationElement->QueryIntAttribute("id", &translation->translationId);

		str = translationElement->Attribute("value");
		sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

		this->translations.push_back(translation);

		translationElement = translationElement->NextSiblingElement("Translation");
	}

	// read scalings
	xmlElement = rootNode->FirstChildElement("Scalings");
	XMLElement *scalingElement = xmlElement->FirstChildElement("Scaling");
	while (scalingElement != NULL)
	{
		Scaling *scaling = new Scaling();

		scalingElement->QueryIntAttribute("id", &scaling->scalingId);
		str = scalingElement->Attribute("value");
		sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

		this->scalings.push_back(scaling);

		scalingElement = scalingElement->NextSiblingElement("Scaling");
	}

	// read rotations
	xmlElement = rootNode->FirstChildElement("Rotations");
	XMLElement *rotationElement = xmlElement->FirstChildElement("Rotation");
	while (rotationElement != NULL)
	{
		Rotation *rotation = new Rotation();

		rotationElement->QueryIntAttribute("id", &rotation->rotationId);
		str = rotationElement->Attribute("value");
		sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

		this->rotations.push_back(rotation);

		rotationElement = rotationElement->NextSiblingElement("Rotation");
	}

	// read meshes
	xmlElement = rootNode->FirstChildElement("Meshes");

	XMLElement *meshElement = xmlElement->FirstChildElement("Mesh");
	while (meshElement != NULL)
	{
		Mesh *mesh = new Mesh();

		meshElement->QueryIntAttribute("id", &mesh->meshId);

		// read projection type
		str = meshElement->Attribute("type");

		if (strcmp(str, "wireframe") == 0)
		{
			mesh->type = WIREFRAME_MESH;
		}
		else
		{
			mesh->type = SOLID_MESH;
		}

		// read mesh transformations
		XMLElement *meshTransformationsElement = meshElement->FirstChildElement("Transformations");
		XMLElement *meshTransformationElement = meshTransformationsElement->FirstChildElement("Transformation");

		while (meshTransformationElement != NULL)
		{
			char transformationType;
			int transformationId;

			str = meshTransformationElement->GetText();
			sscanf(str, "%c %d", &transformationType, &transformationId);

			mesh->transformationTypes.push_back(transformationType);
			mesh->transformationIds.push_back(transformationId);

			meshTransformationElement = meshTransformationElement->NextSiblingElement("Transformation");
		}

		mesh->numberOfTransformations = mesh->transformationIds.size();

		// read mesh faces
		char *row;
		char *cloneStr;
		int v1, v2, v3;
		XMLElement *meshFacesElement = meshElement->FirstChildElement("Faces");
		str = meshFacesElement->GetText();
		cloneStr = strdup(str);

		row = strtok(cloneStr, "\n");
		while (row != NULL)
		{
			int result = sscanf(row, "%d %d %d", &v1, &v2, &v3);

			if (result != EOF)
			{
				mesh->triangles.push_back(Triangle(v1, v2, v3));
			}
			row = strtok(NULL, "\n");
		}
		mesh->numberOfTriangles = mesh->triangles.size();
		this->meshes.push_back(mesh);

		meshElement = meshElement->NextSiblingElement("Mesh");
	}
}

void Scene::assignColorToPixel(int i, int j, Color c)
{
	this->image[i][j].r = c.r;
	this->image[i][j].g = c.g;
	this->image[i][j].b = c.b;
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
	if (this->image.empty())
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			vector<Color> rowOfColors;
			vector<double> rowOfDepths;

			for (int j = 0; j < camera->verRes; j++)
			{
				rowOfColors.push_back(this->backgroundColor);
				rowOfDepths.push_back(1.01);
			}

			this->image.push_back(rowOfColors);
			this->depth.push_back(rowOfDepths);
		}
	}
	else
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			for (int j = 0; j < camera->verRes; j++)
			{
				assignColorToPixel(i, j, this->backgroundColor);
				this->depth[i][j] = 1.01;
				this->depth[i][j] = 1.01;
				this->depth[i][j] = 1.01;
			}
		}
	}
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
	if (value >= 255.0)
		return 255;
	if (value <= 0.0)
		return 0;
	return (int)(value);
}

/*
	Writes contents of image (Color**) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
	ofstream fout;

	fout.open(camera->outputFilename.c_str());

	fout << "P3" << endl;
	fout << "# " << camera->outputFilename << endl;
	fout << camera->horRes << " " << camera->verRes << endl;
	fout << "255" << endl;

	for (int j = camera->verRes - 1; j >= 0; j--)
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			fout << makeBetweenZeroAnd255(this->image[i][j].r) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].g) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].b) << " ";
		}
		fout << endl;
	}
	fout.close();
}

/*
	Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
*/
void Scene::convertPPMToPNG(string ppmFileName)
{
	string command;

	// TODO: Change implementation if necessary.
	command = "./magick convert " + ppmFileName + " " + ppmFileName + ".png";
	system(command.c_str());
}

/*
	Transformations, clipping, culling, rasterization are done here.
*/
void Scene::forwardRenderingPipeline(Camera *camera)
{
	// TODO: Implement this function

	// calculate viewing transform matrices	
	Matrix4 cameraViewportTransformationMatrix = calculateCameraViewportTransformationMatrix(camera);

	//cout << "camera viewport matrix: \n" << cameraViewportTransformationMatrix << endl;
	
	Matrix4 cameraProjectionMatrix = calculateCameraProjectionMatrix(camera);

	//cout << "camera projection matrix: \n" << cameraProjectionMatrix << endl;
	
	Matrix4 cameraTransformationMatrix = calculateCameraTransformationMatrix(camera);

	//cout << "camera transformation matrix: \n" << cameraTransformationMatrix << endl;

	Matrix4 cameraProjectionTransformationMatrix = multiplyMatrixWithMatrix(cameraProjectionMatrix,cameraTransformationMatrix);
	
	for (size_t i = 0; i < this->meshes.size(); ++i) { //for each mesh
		// do the transformations on each vertex
		Matrix4 modelTransformationMatrix = calculateModelTransformationMatrix(*this->meshes[i], this->scalings, this->rotations, this->translations);
		//cout << "model transformation matrix: \n " << modelTransformationMatrix << endl;

		Matrix4 modelProjectionMatrix = multiplyMatrixWithMatrix(cameraProjectionTransformationMatrix,modelTransformationMatrix);
		
		//cout << "modelProjectionMatrix done" << endl;
		
		for(size_t j = 0; j < this->meshes[i]->triangles.size(); ++j) //for each triangle
		{
			//cout << "for each triangle: " << j << endl;
			
			Vec4 vertex0 = Vec4(this->vertices[this->meshes[i]->triangles[j].vertexIds[0]-1]->x,this->vertices[this->meshes[i]->triangles[j].vertexIds[0]-1]->y,this->vertices[this->meshes[i]->triangles[j].vertexIds[0]-1]->z,1,this->vertices[this->meshes[i]->triangles[j].vertexIds[0]-1]->colorId);
			Vec4 vertex1 = Vec4(this->vertices[this->meshes[i]->triangles[j].vertexIds[1]-1]->x,this->vertices[this->meshes[i]->triangles[j].vertexIds[1]-1]->y,this->vertices[this->meshes[i]->triangles[j].vertexIds[1]-1]->z,1,this->vertices[this->meshes[i]->triangles[j].vertexIds[1]-1]->colorId);
			Vec4 vertex2 = Vec4(this->vertices[this->meshes[i]->triangles[j].vertexIds[2]-1]->x,this->vertices[this->meshes[i]->triangles[j].vertexIds[2]-1]->y,this->vertices[this->meshes[i]->triangles[j].vertexIds[2]-1]->z,1,this->vertices[this->meshes[i]->triangles[j].vertexIds[2]-1]->colorId);

			Color color0 = Color(*this->colorsOfVertices[vertex0.colorId-1]);
			Color color1 = Color(*this->colorsOfVertices[vertex1.colorId-1]);
			Color color2 = Color(*this->colorsOfVertices[vertex2.colorId-1]);
			
			Vec4 projected0 = multiplyMatrixWithVec4(modelProjectionMatrix,vertex0);
			Vec4 projected1 = multiplyMatrixWithVec4(modelProjectionMatrix,vertex1);
			Vec4 projected2 = multiplyMatrixWithVec4(modelProjectionMatrix,vertex2);

			//cout << "vertex0: \n" << vertex0 << endl;
			//cout << "projected0: \n" << projected0 << endl;

			if(this->cullingEnabled && isCulled(projected0,projected1,projected2)) //skip if culled
			{
				cout << "culling is enabled and current triangle is backfaced so it got culled" << endl;
				continue;
			}

			if(this->meshes[i]->type == 0) // if wireframe mode
			{
				cout << "in wireframe mode" << endl;
				//do perspective division if necessary
				perspectiveDivision(projected0);
				perspectiveDivision(projected1);
				perspectiveDivision(projected2);
				//cout << "projected0: \n" << projected0 << endl;

				//duplicate the vertexes again colors because clipping one will affect other lines and we don't want that
				Vec4 projected0_copy = Vec4(projected0);
				Vec4 projected1_copy = Vec4(projected1);
				Vec4 projected2_copy = Vec4(projected2);
				//cout << "projected0_copy: \n" << projected0_copy << endl;

				Color color0_copy = Color(color0);
				Color color1_copy = Color(color1);
				Color color2_copy = Color(color2);
				
				//line v0-v1
				bool is01Visible = liangBarsky(projected0, projected1, color0, color1);
				//line v1-v2
				bool is12Visible = liangBarsky( projected1_copy, projected2, color1_copy, color2);
				//line v2-v0
				bool is20Visible = liangBarsky( projected2_copy, projected0_copy, color2_copy, color0_copy);

				//we finally have our clipped lines, if they are visible, apply viewport transformation to their vertices to get their coordinates on viewport
				if(is01Visible)
				{
					//cout << "01 visible" << endl;
					Vec4 viewported0 = multiplyMatrixWithVec4(cameraViewportTransformationMatrix, projected0);
					Vec4 viewported1 = multiplyMatrixWithVec4(cameraViewportTransformationMatrix, projected1);
					midpoint(this->image, viewported0, viewported1, color0, color1);
				}
				if(is12Visible)
				{
					//cout << "12 visible" << endl;
					Vec4 viewported1_copy = multiplyMatrixWithVec4(cameraViewportTransformationMatrix, projected1_copy);
					Vec4 viewported2 = multiplyMatrixWithVec4(cameraViewportTransformationMatrix, projected2);
					midpoint(this->image, viewported1_copy, viewported2, color1_copy, color2);
				}
				if(is20Visible)
				{
					//cout << "20 visible" << endl;
					Vec4 viewported2_copy = multiplyMatrixWithVec4(cameraViewportTransformationMatrix, projected2_copy);
					Vec4 viewported0_copy = multiplyMatrixWithVec4(cameraViewportTransformationMatrix, projected0_copy);
					midpoint(this->image, viewported2_copy, viewported0_copy, color2_copy, color0_copy);
				}
			}
			else //else solid mode
			{
				
			}
			
		}
		//do clipping and culling	
		//do rasterization
	}
	
}
