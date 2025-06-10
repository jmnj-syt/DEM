
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
	bool m_bLeftDown, m_bRightDown, m_bMiddleDown;

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
	
	void AddVertex(int x, int y, const std::vector<float>& demData, float scale);
	
	//void AddVertex(int x, int y, float demData, float scale);
	void LoadDEM(const char* filename);
	GLuint CompileShader(const char* source, GLenum type);
	void LoadTIFFWithLibTiff(const char* path);

	// 生成的消息映射函数
protected:
	DECLARE_MESSAGE_MAP()
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnDestroy();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnMButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
public:
	CString CTerrainView::OpenFileDialog();
	afx_msg void Onfile();		// 打开文件菜单项处理函数
	void ClearScene();         // 清除当前场景
	bool m_showTexture = true;        // 是否显示纹理
	bool m_showColorRamp = false;      // 是否显示分层颜色
	std::vector<float> m_colorRampValues;  // 分层颜色的高度值
	std::vector<glm::vec3> m_colorRampColors;  // 对应的颜色值

	afx_msg void OnViewTexture();
	afx_msg void OnViewColorRamp();
	afx_msg void OnUpdateViewTexture(CCmdUI* pCmdUI);
	afx_msg void OnUpdateViewColorRamp(CCmdUI* pCmdUI);
	// 函数声明
	void InitializeColorRamp();  // 初始化颜色表
	void UpdateColorRamp();      // 更新颜色表

	void SetTextureVisibility(bool visible); // 设置纹理可见性
	void ApplyColorRamp(); // 应用颜色表到DEM
	void AddColoredVertex(int x, int y, const std::vector<float>& demData,
		float scale, float minHeight, float range);

	void CTerrainView::TransformCoordinates(double& x, double& y);
	afx_msg void OnViewProfile();

	// 在类定义中添加以下成员变量和函数
protected:
	glm::vec3 m_profilePoint1;  // 剖面起点
	glm::vec3 m_profilePoint2;  // 剖面终点
	bool m_selectingProfile;    // 是否正在选择剖面点
	int m_profilePointSelected; // 当前选择的点(0-未选,1-第一个点,2-第二个点)
	std::vector<glm::vec3> m_profileData; // 剖面数据
	bool m_showProfile;
	// 剖面图窗口相关
	HWND m_hProfileWnd;
	static LRESULT CALLBACK ProfileWndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
	void DrawProfileInView(CDC* pDC);      // 在主视图中绘制剖面图

public:
	void StartProfileSelection(); // 开始选择剖面点
	void CalculateProfile();      // 计算剖面数据
	void ClearProfile();          // 清除剖面
};

#ifndef _DEBUG  // TerrainView.cpp 中的调试版本
inline CTerrainDoc* CTerrainView::GetDocument() const
{
	return reinterpret_cast<CTerrainDoc*>(m_pDocument);
}
#endif

