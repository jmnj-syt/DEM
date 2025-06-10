// TerrainView.h: CTerrainView 类的接口
//

#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include "gdal_priv.h"
#include "ogr_spatialref.h"

class CTerrainView : public CView
{
protected: // 仅从序列化创建
	CTerrainView();
	DECLARE_DYNCREATE(CTerrainView)

	HGLRC m_hRC;
	CDC* m_pDC;

	GLuint m_terrainVAO, m_terrainVBO, m_textureID;
	GLuint m_shaderProgram;

	std::vector<float> m_vertices;
	int m_terrainWidth, m_terrainHeight;

	glm::vec3 m_cameraPos;
	glm::vec3 m_cameraFront;
	glm::vec3 m_cameraUp;

	CPoint m_lastMousePos;
	// 用于存储鼠标上一次的位置
	// 当鼠标发生移动事件时，会将当前鼠标位置保存到这个变量中，
	// 以便后续计算鼠标移动的偏移量，从而实现相应的交互操作，如平移、旋转等
	bool m_bLeftDown, m_bRightDown, m_bMiddleDown;  // 左键、右键、中键的标志变量

	double m_demGeoTransform[6];    // DEM地理转换参数
	double m_textureGeoTransform[6]; // 纹理地理转换参数
	int m_texturePixelWidth;            // 纹理影像像素宽度
	int m_texturePixelHeight;           // 纹理影像像素高度
	double m_textureWidth;// 纹理影像实际宽度
	double m_textureHeight;// 纹理影像实际高度
	glm::dvec2 m_terrainCenter; // 地形中心点的投影坐标（米）
	bool ConvertGeoTransformToProjected(double* geoTransform);

	float m_scale; // 高程缩放比例
	std::vector<float> m_demData; // 保存DEM原始数据
	void RegenerateTerrainVertices(); // 根据当前比例重新生成地形顶点
	GDALDataType m_dataType;

	bool m_bTextureEnabled; // 添加纹理启用标志
	GLint m_useTextureLoc; // 新增：useTexture的uniform位置
	//bool m_is16BitLoc;
// 特性
public:
	CTerrainDoc* GetDocument() const;

	// 操作
public:

	// 重写
public:
	virtual void OnDraw(CDC* pDC);  // 重写以绘制该视图
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);

	// 实现
public:
	virtual ~CTerrainView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

	void AddVertex(int x, int y, const std::vector<float>& demData, float scale);// 增加顶点方法

	//void AddVertex(int x, int y, float demData, float scale);
	void LoadDEM(const char* filename);  // 导入dem方法
	GLuint CompileShader(const char* source, GLenum type);
	void LoadTIFFWithLibTiff(const char* path);  //对不同格式的 TIFF 图像进行相应处理，确保能在 OpenGL 中正确显示。

	// 生成的消息映射函数
protected:
	DECLARE_MESSAGE_MAP()
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnDestroy();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);  //左键按下
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);  //右键按下
	afx_msg void OnMButtonDown(UINT nFlags, CPoint point);  //中键按下
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnFileOpen();
public:
	afx_msg void OnScaleEnhance();
	afx_msg void OnScaleReduce();
	afx_msg void OnCloseTexture();
	afx_msg void OnOpenTexture();
};

#ifndef _DEBUG  // TerrainView.cpp 中的调试版本
inline CTerrainDoc* CTerrainView::GetDocument() const
{
	return reinterpret_cast<CTerrainDoc*>(m_pDocument);
}
#endif

