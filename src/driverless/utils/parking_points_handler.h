#ifndef PARKING_POINTS_HANDLER_
#define PARKING_POINTS_HANDLER_

#include <tinyxml2.h>
#include <iostream>
using namespace std;

bool loadParkingPoints(const std::string& file)
{
	using namespace tinyxml2;
	tinyxml2::XMLDocument Doc;  
	XMLError res = Doc.LoadFile(file.c_str());
	
	if(XML_ERROR_FILE_NOT_FOUND == res)
	{
		cout << "parking points file: "<< file << "not exist!" << endl;
		return false;
	}
	else if(XML_SUCCESS != res)
	{
		cout << "parking points file: "<< file << " parse error!" << endl;
		return false;
	}
	tinyxml2::XMLElement *pRoot=Doc.RootElement();//根节点
	
	tinyxml2::XMLElement *pPoint=pRoot->FirstChildElement(); //一级子节点
	while (pPoint)
	{
		uint32_t id = pPoint->Unsigned64Attribute("id");
		uint32_t index = pPoint->Unsigned64Attribute("index");
		float duration = pPoint->FloatAttribute("duration");
		cout << id << "\t" << index << "\t" << duration << endl;
		//获取下一层子节点
		tinyxml2::XMLElement *pPointEle = pPoint->FirstChildElement();
		while(pPointEle)
		{
			std::cout << pPointEle->GetText() << endl;
			//转到下一子节点
			pPointEle = pPointEle->NextSiblingElement();
		}
		
		//转到下一子节点
		pPoint = pPoint->NextSiblingElement();  
	}
}

bool createParkingPointsXml(const std::string& file)
{
	tinyxml2::XMLDocument doc;

	//1.添加声明
	tinyxml2::XMLDeclaration* declaration = doc.NewDeclaration();
	doc.InsertFirstChild(declaration); //在最前插入声明

	//2.创建根节点
	tinyxml2::XMLElement* root = doc.NewElement("ParkingPoints");
	doc.InsertEndChild(root);  //在最后插入根节点

	//3.创建子节点,并插入父节点
	tinyxml2::XMLElement* descriptionEle = doc.NewElement("Description");
	root->InsertEndChild(descriptionEle);
	
	tinyxml2::XMLElement* idElement = doc.NewElement("id");
	descriptionEle->InsertEndChild(idElement);
	idElement->InsertEndChild(doc.NewText("the sequence of the parking point"));
	
	tinyxml2::XMLElement* indexElement = doc.NewElement("index");
	descriptionEle->InsertEndChild(indexElement);
	indexElement->InsertEndChild(doc.NewText("the parking point position in global path"));
	
	tinyxml2::XMLElement* durationElement = doc.NewElement("duration");
	descriptionEle->InsertEndChild(durationElement);
	durationElement->InsertEndChild(doc.NewText("parking time(s), 0 for destination"));
	
	tinyxml2::XMLElement* addEle = doc.NewElement("add");
	descriptionEle->InsertEndChild(addEle);
	addEle->InsertEndChild(doc.NewText("To add a parking point manually, please follow the format below"));

	tinyxml2::XMLElement* pointElement = doc.NewElement("ParkingPoint");
	root->InsertEndChild(pointElement); //在最后插入节点
	
	//4.为子节点增加属性
	pointElement->SetAttribute("id", "0");
	pointElement->SetAttribute("index", "500");
	pointElement->SetAttribute("duration", "40");

	//6.保存xml文件
	doc.SaveFile(file.c_str());
}

#endif







