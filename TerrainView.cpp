
// TerrainView.cpp: CTerrainView 类的实现
//

#include "pch.h"
#include "framework.h"
// SHARED_HANDLERS 可以在实现预览、缩略图和搜索筛选器句柄的
// ATL 项目中进行定义，并允许与该项目共享文档代码。
#ifndef SHARED_HANDLERS
#include "Terrain.h"
#endif

#include "TerrainDoc.h"
#include "TerrainView.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "tiffio.h"

#include <algorithm>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CTerrainView

IMPLEMENT_DYNCREATE(CTerrainView, CView)

BEGIN_MESSAGE_MAP(CTerrainView, CView)
	// 标准打印命令
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CView::OnFilePrintPreview)
	ON_WM_CREATE()
	ON_WM_DESTROY()
	ON_WM_SIZE()
	ON_WM_LBUTTONDOWN()
	ON_WM_RBUTTONDOWN()
	ON_WM_MBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_RBUTTONUP()
	ON_WM_MBUTTONUP()
	ON_WM_MOUSEMOVE()
	ON_WM_MOUSEWHEEL()
	ON_COMMAND(ID_VIEW_COLORRAMP, &CTerrainView::OnViewColorRamp)
	ON_COMMAND(ID_VIEW_TEXTURE, &CTerrainView::OnViewTexture)
	ON_UPDATE_COMMAND_UI(ID_VIEW_TEXTURE, &CTerrainView::OnUpdateViewTexture)
	ON_UPDATE_COMMAND_UI(ID_VIEW_COLORRAMP, &CTerrainView::OnUpdateViewColorRamp)
	ON_COMMAND(ID_file, &CTerrainView::Onfile)
	ON_COMMAND(ID_VIEW_PROFILE, &CTerrainView::OnViewProfile)
END_MESSAGE_MAP()

// CTerrainView 构造/析构


#include <tiffio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>


#include <assert.h>

typedef unsigned __int16 uint16_t;
typedef unsigned unsigned char uint8_t;

#include "gdal_priv.h"
#include <cmath>

#include "gdal_priv.h"
#include <algorithm>


CTerrainView::CTerrainView() : m_hRC(NULL), m_pDC(NULL),
m_cameraPos(0.0f, 50.0f, 50.0f), m_cameraFront(0.0f, -0.5f, -1.0f),
m_cameraUp(0.0f, 1.0f, 0.0f),
m_showTexture(true), m_showColorRamp(false), m_scale(0.01f),
m_selectingProfile(false),
m_profilePointSelected(0),
m_hProfileWnd(NULL)
{
	InitializeColorRamp();
}

CTerrainView::~CTerrainView()
{
}

BOOL CTerrainView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: 在此处通过修改
	//  CREATESTRUCT cs 来修改窗口类或样式

	return CView::PreCreateWindow(cs);
}

// CTerrainView 绘图

void CTerrainView::OnDraw(CDC* pDC)
{
    CTerrainDoc* pDoc = GetDocument();
    ASSERT_VALID(pDoc);
    if (!pDoc)
        return;

    // 激活OpenGL上下文
    wglMakeCurrent(m_pDC->GetSafeHdc(), m_hRC);

    // 清除颜色缓冲区和深度缓冲区
    glClearColor(0.5f, 0.5f, 0.8f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // 获取窗口大小
    CRect rcClient;
    GetClientRect(&rcClient);

    // 设定透视投影
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)rcClient.Width() / rcClient.Height(), 0.1f, 1000.0f);

    // 设定视点和视线
    glm::mat4 view = glm::lookAt(m_cameraPos, m_cameraPos + m_cameraFront, m_cameraUp);
    glm::mat4 model = glm::mat4(1.0f);

    // 使用着色器程序
    glUseProgram(m_shaderProgram);

	glUniform1i(glGetUniformLocation(m_shaderProgram, "showTexture"), m_showTexture);
	glUniform1i(glGetUniformLocation(m_shaderProgram, "showColorRamp"), m_showColorRamp);

    // 设置投影矩阵、视图矩阵和模型矩阵
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "model"),
        1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "view"),
        1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "projection"),
        1, GL_FALSE, glm::value_ptr(projection));

    // 根据m_showTexture决定是否绑定纹理
    if (m_showTexture)
    {
        // 激活纹理单元
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_textureID);
        glUniform1i(glGetUniformLocation(m_shaderProgram, "texture1"), 0);

        // 传递数据类型信息
        GLboolean is16Bit = (m_dataType == GDT_UInt16 || m_dataType == GDT_Int16);
        glUniform1i(glGetUniformLocation(m_shaderProgram, "is16Bit"), is16Bit);
    }

    // 根据m_showColorRamp决定是否应用颜色表
    if (m_showColorRamp)
    {
		// 分层设色模式 - 使用颜色属性
		glBindVertexArray(m_terrainVAO);
		glBindBuffer(GL_ARRAY_BUFFER, m_terrainVBO);

		// 顶点属性布局：位置(3) + 纹理坐标(2) + 颜色(3)
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(5 * sizeof(float)));
		glEnableVertexAttribArray(2);
    }
    else
    {
		// 纹理或默认模式 - 不使用颜色属性
		glBindVertexArray(m_terrainVAO);
		glBindBuffer(GL_ARRAY_BUFFER, m_terrainVBO);

		// 顶点属性布局：位置(3) + 纹理坐标(2)
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);

		// 禁用颜色属性
		glDisableVertexAttribArray(2);
    }



    // 绘制地形
    if (m_showTexture)
    {
        // 绑定纹理
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_textureID);
        glUniform1i(glGetUniformLocation(m_shaderProgram, "texture1"), 0);
    }

    // 绘制顶点
    glDrawArrays(GL_TRIANGLES, 0, m_vertices.size() / (m_showColorRamp ? 8 : 5));

    // 解绑VAO
    glBindVertexArray(0);

    // 交换缓冲区
    SwapBuffers(m_pDC->GetSafeHdc());

	// 如果剖面数据存在且需要显示，则绘制
	if (m_showProfile && !m_profileData.empty()) {
		DrawProfileInView(pDC);
	}
}


// CTerrainView 打印

BOOL CTerrainView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// 默认准备
	return DoPreparePrinting(pInfo);
}

void CTerrainView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 添加额外的打印前进行的初始化过程
}

void CTerrainView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 添加打印后进行的清理过程
}


// CTerrainView 诊断

#ifdef _DEBUG
void CTerrainView::AssertValid() const
{
	CView::AssertValid();
}

void CTerrainView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CTerrainDoc* CTerrainView::GetDocument() const // 非调试版本是内联的
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CTerrainDoc)));
	return (CTerrainDoc*)m_pDocument;
}
#endif //_DEBUG


// CTerrainView 消息处理程序
int CTerrainView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CView::OnCreate(lpCreateStruct) == -1) return -1;

	PIXELFORMATDESCRIPTOR pfd = {
		sizeof(PIXELFORMATDESCRIPTOR), 1,
		PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
		PFD_TYPE_RGBA, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		24, 8, 0, PFD_MAIN_PLANE, 0, 0, 0, 0
	};

	m_pDC = new CClientDC(this);
	int pf = ChoosePixelFormat(m_pDC->GetSafeHdc(), &pfd);
	SetPixelFormat(m_pDC->GetSafeHdc(), pf, &pfd);

	m_hRC = wglCreateContext(m_pDC->GetSafeHdc());
	wglMakeCurrent(m_pDC->GetSafeHdc(), m_hRC);

	gladLoadGL();

	// 顶点着色器（确保位置和纹理坐标匹配）
	// 修改后的顶点着色器
	const char* vs = R"(
    #version 460 core
    layout(location = 0) in vec3 aPos;
    layout(location = 1) in vec2 aTexCoord;
	layout(location = 2) in vec3 aColor;  // 添加颜色输入
    out vec2 TexCoord; // 明确声明输出变量
	out vec3 Color;  // 添加颜色输出
    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;
    
    void main() {
        // 增加Y轴翻转（GDAL坐标系与OpenGL纹理坐标系不同）
        TexCoord = vec2(aTexCoord.x, 1.0 - aTexCoord.y); 
		Color = aColor;  // 传递颜色到片段着色器
        gl_Position = projection * view * model * vec4(aPos, 1.0);
    })";

	// 片段着色器（添加Gamma校正开关）
	// 修正后的片段着色器
	const char* fs = R"(
    #version 460 core
    in vec2 TexCoord;
    in vec3 Color;  // 接收顶点颜色输入
    out vec4 FragColor;
    
    // 纹理相关uniform
    uniform sampler2D texture8Bit;    // 8位/浮点纹理
    uniform usampler2D texture16Bit;  // 16位无符号整数纹理
    uniform bool is16Bit;             // 是否为16位数据
    
    // 显示模式控制uniform
    uniform bool showTexture;         // 是否显示纹理
    uniform bool showColorRamp;       // 是否显示分层设色

    void main() {
        // 优先检查分层设色模式
        if (showColorRamp) {
            // 直接使用顶点颜色
            FragColor = vec4(Color, 1.0);
            return;
        }
        
        // 然后是纹理模式
        if (showTexture) {
            if (is16Bit) {
                // 读取16位无符号整数并归一化
                uint rawValue = texture(texture16Bit, TexCoord).r;
                float normalizedValue = float(rawValue) / 65535.0;
                FragColor = vec4(vec3(normalizedValue), 1.0);
            } else {
                // 直接读取8位/浮点数据
                FragColor = texture(texture8Bit, TexCoord);
            }
            return;
        }
        
        // 默认显示（当既不显示纹理也不显示分层设色时）
        FragColor = vec4(0.5, 0.5, 0.8, 1.0); // 使用背景色
    })";

	GLuint vertexShader = CompileShader(vs, GL_VERTEX_SHADER);
	GLuint fragmentShader = CompileShader(fs, GL_FRAGMENT_SHADER);

	m_shaderProgram = glCreateProgram();
	glAttachShader(m_shaderProgram, vertexShader);
	glAttachShader(m_shaderProgram, fragmentShader);
	glLinkProgram(m_shaderProgram);

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);

	//为了简化调试，把数据路径固定了，放在data目录下了。自己考虑选择打开任意文件。
	CString strTemp = theApp.GetExePath() + "\\data\\";
	CString strDem = OpenFileDialog();
	CString strTexture = OpenFileDialog();


	//下面两个函数的顺序不能乱，要先读取影像，后续读取dem、处理dem顶点的纹理坐标时需要影像的信息
	LoadTIFFWithLibTiff(strTexture.GetBuffer());//加载地面影像作为纹理
	LoadDEM(strDem.GetBuffer());//加载dem，计算dem顶点坐标、构建规则三角网、计算纹理坐标

	// 在初始化时设置纹理单元
	glUseProgram(m_shaderProgram);
	// 在OnCreate函数中获取Uniform位置

	//考虑到将在影像读取阶段将16位的转换成8位，因此不需要考虑opengl对16位的支持，下面内容要进行调整
//	m_is16BitLoc = glGetUniformLocation(m_shaderProgram, "is16Bit");

	glUniform1i(glGetUniformLocation(m_shaderProgram, "texture1"), 0);
	//GLboolean is16Bit = (m_dataType == GDT_UInt16 || m_dataType == GDT_Int16);
	//glUniform1i(m_is16BitLoc, is16Bit);

	// 绑定纹理单元
	glUniform1i(glGetUniformLocation(m_shaderProgram, "texture8Bit"), 0);
	glUniform1i(glGetUniformLocation(m_shaderProgram, "texture16Bit"), 0);
	//glUniform1i(glGetUniformLocation(m_shaderProgram, "is16Bit"),(m_dataType == GDT_UInt16));

	// 确保VAO配置正确
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

	m_cameraPos = glm::vec3(0.0f, m_terrainHeight * 0.2f, m_terrainWidth * 0.3f);//相机(视点)位置
	m_cameraFront = glm::normalize(glm::vec3(0.0f, -0.5f, -1.0f));//实现前方位置

	// 检查OpenGL版本
	const GLubyte* version = glGetString(GL_VERSION);
	CString msg;
	msg.Format(_T("OpenGL版本: %hs"), version);
	OutputDebugString(msg);

	return 0;
}

void CTerrainView::OnDestroy()
{
	wglDeleteContext(m_hRC);
	delete m_pDC;
	CView::OnDestroy();
}

void CTerrainView::OnSize(UINT nType, int cx, int cy)
{
	CView::OnSize(nType, cx, cy);
	if (cx == 0 || cy == 0) return;
	glViewport(0, 0, cx, cy);
}

void CTerrainView::LoadDEM(const char* filename)
{
	GDALAllRegister();

	CPLSetConfigOption("SHAPE_ENCODING", "");//支持中文
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");

	// 清理旧地形数据
	if (glIsVertexArray(m_terrainVAO)) glDeleteVertexArrays(1, &m_terrainVAO);
	if (glIsBuffer(m_terrainVBO)) glDeleteBuffers(1, &m_terrainVBO);
	m_vertices.clear();

	GDALDataset* poDataset = (GDALDataset*)GDALOpen(filename, GA_ReadOnly);
	if (!poDataset) return;

	// 获取地理转换参数
	if (poDataset->GetGeoTransform(m_demGeoTransform) != CE_None) {
		AfxMessageBox(_T("DEM文件缺少地理参考信息!"));
		GDALClose(poDataset);
		return;
	}

	// 将WGS84坐标转换为CGCS2000 39带投影坐标
	if (!ConvertGeoTransformToProjected(m_demGeoTransform)) {
		AfxMessageBox(_T("DEM坐标转换失败"));
		GDALClose(poDataset);
		return;
	}

	m_terrainWidth = poDataset->GetRasterXSize();
	m_terrainHeight = poDataset->GetRasterYSize();
	GDALRasterBand* poBand = poDataset->GetRasterBand(1);

	m_demData.resize(m_terrainWidth * m_terrainHeight);
	poBand->RasterIO(GF_Read, 0, 0, m_terrainWidth, m_terrainHeight,
		m_demData.data(), m_terrainWidth, m_terrainHeight, GDT_Float32, 0, 0);

	// 计算地形边界
	double minX = m_demGeoTransform[0];
	double maxX = m_demGeoTransform[0] + m_terrainWidth * m_demGeoTransform[1];
	double minY = m_demGeoTransform[3] + m_terrainHeight * m_demGeoTransform[5]; // 注意：m_demGeoTransform[5]是负值
	double maxY = m_demGeoTransform[3];

	// 计算地形中心点（投影坐标，单位：米）
	m_terrainCenter.x = (minX + maxX) / 2.0;
	m_terrainCenter.y = (minY + maxY) / 2.0;

	// 使用当前缩放比例生成顶点
	RegenerateTerrainVertices();

	glGenVertexArrays(1, &m_terrainVAO);
	glGenBuffers(1, &m_terrainVBO);

	glBindVertexArray(m_terrainVAO);
	glBindBuffer(GL_ARRAY_BUFFER, m_terrainVBO);
	glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(float),
		m_vertices.data(), GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
		(void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	GDALClose(poDataset);
}

void CTerrainView::AddVertex(int x, int y, const std::vector<float>& demData, float scale) {
	// x，y行列号，转换为真实地理坐标
	double geoX = m_demGeoTransform[0] + x * m_demGeoTransform[1] + y * m_demGeoTransform[2];
	double geoY = m_demGeoTransform[3] + x * m_demGeoTransform[4] + y * m_demGeoTransform[5];

	// 计算相对于地形中心的偏移量（单位：米）
	double offsetX = geoX - m_terrainCenter.x;
	double offsetY = geoY - m_terrainCenter.y;

	// 精确计算纹理坐标
	double texPixelX = (geoX - m_textureGeoTransform[0]) / m_textureGeoTransform[1];
	double texPixelY = (geoY - m_textureGeoTransform[3]) / m_textureGeoTransform[5];

	// 归一化处理
	float u = static_cast<float>(texPixelX / (m_texturePixelWidth - 1));
	float v = 1.0f - static_cast<float>(texPixelY / (m_texturePixelHeight - 1));

	// 边界约束
	u = std::clamp(u, 0.0f, 1.0f);
	v = std::clamp(v, 0.0f, 1.0f);

	// 顶点坐标计算
	float xPos = offsetX * 0.01;
	float zPos = offsetY * 0.01;
	float yPos = demData[y * m_terrainWidth + x] * 0.1;

	// 添加顶点数据
	m_vertices.insert(m_vertices.end(), { xPos, yPos, zPos, u, v });
}


//TIFF的影像的波段数、位数等很多，有的格式opengl并不直接支持，需要分情况处理
//目前的纹理支持单波段8位、多波段16位、多波段8位等格式tif，其他情况还需要遇到后再测试
//考虑到opengl对16位的支持有限（软件版本、硬件配置等），需要将其先转换成8位。因此不涉及opengl对16位的支持
//但是影像解码转换需要占用不少时间
void CTerrainView::LoadTIFFWithLibTiff(const char* path)
{
	//在gdal这里处理两种情况，1.灰度图像转24彩色  2.16位多波段转8位多波段 
	//其余的 8位深的，下面libtiff处理应无问题。
	//两种情况共用纹理处理代码
	
	////////////////////////////////////////////////////////
	//一，gdal获得部分信息，处理上述两种情况，转为8位rgb、rgba
	GDALAllRegister();

	CPLSetConfigOption("SHAPE_ENCODING", "");//支持中文
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");

	// 打开数据集
	GDALDataset* poDataset = static_cast<GDALDataset*>(GDALOpen(path, GA_ReadOnly));
	if (!poDataset) {
		AfxMessageBox(_T("无法打开纹理文件"));
		return;
	}
	if (poDataset->GetGeoTransform(m_textureGeoTransform) != CE_None) {
		AfxMessageBox(_T("纹理文件缺少地理参考信息!"));
		GDALClose(poDataset);
		return;
	}
	uint32 width, height;

	// 将WGS84坐标转换为CGCS2000 39带投影坐标
	if (!ConvertGeoTransformToProjected(m_textureGeoTransform)) {
		AfxMessageBox(_T("纹理坐标转换失败"));
		GDALClose(poDataset);
		return;
	}
	
	m_texturePixelWidth = poDataset->GetRasterXSize();
	m_texturePixelHeight = poDataset->GetRasterYSize();
	int bandCount = poDataset->GetRasterCount();
	width = m_texturePixelWidth;
	height = m_texturePixelHeight;

	GDALRasterBand* poBand = poDataset->GetRasterBand(1);
	GDALDataType type = poBand->GetRasterDataType();
	int pixelSize = GDALGetDataTypeSizeBytes(type);

	uint16 bitsPerSample = GDALGetDataTypeSizeBits(type);
	GLenum format, internalFormat, dataType;

	
	pixelSize = 1;
	uint16 channels = 3;
	const tmsize_t stride = width * pixelSize * channels;
	std::vector<uint8_t> buffer(stride * height);

	bool bDone = false;//是否已被gdal处理
	if (bandCount == 1 && type == GDT_Byte)//单波段、8位灰度图像转24位彩色
	{
		bDone = true;
		// 读取灰度数据
		uint8_t* grayData = (uint8_t*)CPLMalloc(width * height);
		poBand->RasterIO(GF_Read, 0, 0, width, height, grayData, width, height, GDT_Byte, 0, 0);

		// 检查输入图像是否包含颜色表
		const GDALColorTable* colorTable = poBand->GetColorTable();
		std::vector<uint8_t> rLUT(256), gLUT(256), bLUT(256);

		if (colorTable) {
			// 使用输入图像的颜色表
			for (int i = 0; i < colorTable->GetColorEntryCount(); ++i) {
				const GDALColorEntry* entry = colorTable->GetColorEntry(i);
				rLUT[i] = static_cast<uint8_t>(entry->c1);
				gLUT[i] = static_cast<uint8_t>(entry->c2);
				bLUT[i] = static_cast<uint8_t>(entry->c3);
			}
		}
		else {
			// 创建灰度颜色表（默认黑白）
			for (int i = 0; i < 256; ++i) {
				rLUT[i] = gLUT[i] = bLUT[i] = static_cast<uint8_t>(i);
			}
		}

		for (int i = 0; i < width * height; ++i) {
			const uint8_t val = grayData[i];
			buffer[i * 3] = rLUT[val];
			buffer[i * 3 + 1] = gLUT[val];
			buffer[i * 3 + 2] = bLUT[val];
		}
	}
	else if (type == GDT_UInt16 || type == GDT_Int16)//16位多波段转8位多波段
	{
		bDone = true;
		
		for (int iBand = 0; iBand < bandCount; iBand++)
		{
			uint16_t* srcData = (uint16_t*)malloc(sizeof(uint16_t) * width * height);
			poDataset->GetRasterBand(iBand + 1)->RasterIO(GF_Read, 0, 0, width, height, srcData, width, height, GDT_UInt16, 0, 0);

			// 统计原图的最小最大值
			int src_min = 65535, src_max = 0;
			for (int i = 0; i < width * height; i++)
			{
				uint16_t val = srcData[i];
				if (val < src_min) src_min = val;
				if (val > src_max) src_max = val;
			}

			// 计算直方图
			double* frequency_val = (double*)calloc(src_max + 1, sizeof(double));
			for (int i = 0; i < width * height; i++)
				frequency_val[srcData[i]] += 1.0;

			for (int i = 0; i <= src_max; i++)
				frequency_val[i] /= (width * height);

			// 计算累积频率
			double* accum_freq = (double*)calloc(src_max + 1, sizeof(double));
			accum_freq[0] = frequency_val[0];
			for (int i = 1; i <= src_max; i++)
				accum_freq[i] = accum_freq[i - 1] + frequency_val[i];

			// 确定截断点：2%和98%
			int minVal = 0, maxVal = src_max;
			for (int i = 0; i <= src_max; i++)
				if (accum_freq[i] > 0.02) { minVal = i; break; }

			for (int i = src_max; i >= 0; i--)
				if (accum_freq[i] < 0.98) { maxVal = i; break; }

			// 处理像素转换
			//uint8_t* dstData = (uint8_t*)malloc(src_width * src_height);
			for (int i = 0; i < width * height; i++)
			{
				uint16_t val = srcData[i];


				long nPixelPos = i * bandCount + iBand;
				if (val < minVal)
					buffer[nPixelPos] = 0;
				else if (val > maxVal)
					buffer[nPixelPos] = 255;
				else
				{
					double scaled = (double)(val - minVal) / (maxVal - minVal);
					scaled = pow(scaled, 0.5); // Gamma校正
					buffer[nPixelPos] = (uint8_t)(scaled * 255.0);
				}
			}
			
			free(srcData);
			free(frequency_val);
			free(accum_freq);
		}
	}
	//按8位多波段来设定下述参数
	internalFormat = GL_RGB;
	format = GL_RGB;
	dataType = GL_UNSIGNED_BYTE;
	
	GDALClose(poDataset);
	////////////////////////////////////////////////////
	//二、用libtiff处理 上面gdal处理之外的情况，无法处理的影像要排除
	// 1. 打开TIFF文件
	if(!bDone)
	{
		TIFF* tif = TIFFOpen(path, "r");
		if (!tif) {
		//	std::cerr << "无法打开TIFF文件: " << path << std::endl;
			return;
		}

		// 2. 读取基本参数
		uint16 sampleFormat;
	// 	TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
	// 	TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
	// 	TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &channels);
		TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bitsPerSample);
		TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLEFORMAT, &sampleFormat);

		m_texturePixelWidth = width;
		m_texturePixelHeight = height;

		// 3. 验证支持的格式
		const bool isSupported = (channels == 1 || channels == 3 || channels == 4) &&
			(bitsPerSample == 8 || bitsPerSample == 16 || bitsPerSample == 32) &&
			(sampleFormat == SAMPLEFORMAT_UINT || sampleFormat == SAMPLEFORMAT_IEEEFP);

		if (!isSupported) {
			AfxMessageBox("不支持的TIFF格式纹理",MB_OK);
	// 			<< channels << "通道 "
	// 			<< bitsPerSample << "位 "
	// 			<< (sampleFormat == SAMPLEFORMAT_IEEEFP ? "浮点" : "整型")
	// 			<< std::endl;
			TIFFClose(tif);
			return;
		}

		// 4. 确定OpenGL格式
	
		switch (channels) {
		case 1: format = GL_RED; break;
		case 3: format = GL_RGB; break;
		case 4: format = GL_RGBA; break;
		}

		switch (bitsPerSample) {
		case 8:
			dataType = GL_UNSIGNED_BYTE;
			internalFormat = (channels == 1) ? GL_R8 :
				(channels == 3) ? GL_RGB8 :
				GL_RGBA8;
			break;
		case 16:
			dataType = GL_UNSIGNED_SHORT;
			internalFormat = (channels == 1) ? GL_R16 :
				(channels == 3) ? GL_RGB16 :
				GL_RGBA16;
			break;
		case 32:
			dataType = (sampleFormat == SAMPLEFORMAT_IEEEFP) ? GL_FLOAT : GL_UNSIGNED_INT;
			internalFormat = (channels == 1) ? GL_R32F :
				(channels == 3) ? GL_RGB32F :
				GL_RGBA32F;
			break;
		}

		// 5. 按行扫描、处理像素数据
		for (uint32 row = 0; row < height; ++row)
		{
			if (TIFFReadScanline(tif, &buffer[row * stride], row) == -1) 
			{
				//std::cerr << "读取扫描线失败: 行 " << row << std::endl;
				TIFFClose(tif);
				return;
			}
		}
		TIFFClose(tif);
	}

	//三、下面是纹理处理用上面获得的影像信息，进行纹理设定
	// 7. 创建OpenGL纹理
	glGenTextures(1, &m_textureID);
	glBindTexture(GL_TEXTURE_2D, m_textureID);

	// 8. 设置对齐方式
	const uint32 bytesPerPixel = (bitsPerSample / 8) * channels;
	glPixelStorei(GL_UNPACK_ALIGNMENT, (bytesPerPixel % 4) ? 1 : 4);

	// 9. 上传纹理数据
	glTexImage2D(GL_TEXTURE_2D, 0, internalFormat,width, height, 0,format, dataType, buffer.data());

	// 10. 设置默认纹理参数
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
}

// 鼠标交互实现
void CTerrainView::OnLButtonDown(UINT nFlags, CPoint point) {
	if (m_selectingProfile)
	{
		// 将屏幕坐标转换为OpenGL世界坐标
		CRect rcClient;
		GetClientRect(&rcClient);

		// 获取深度缓冲区
		GLfloat depth;
		wglMakeCurrent(m_pDC->GetSafeHdc(), m_hRC);
		glReadPixels(point.x, rcClient.Height() - point.y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);

		if (depth < 1.0f) // 确保点击到了地形
		{
			// 将屏幕坐标转换为世界坐标
			GLfloat winX = (GLfloat)point.x;
			GLfloat winY = (GLfloat)(rcClient.Height() - point.y);
			GLfloat winZ = depth;

			glm::mat4 view = glm::lookAt(m_cameraPos, m_cameraPos + m_cameraFront, m_cameraUp);
			glm::mat4 projection = glm::perspective(glm::radians(45.0f),
				(float)rcClient.Width() / rcClient.Height(), 0.1f, 1000.0f);

			glm::vec4 viewport(0, 0, rcClient.Width(), rcClient.Height());
			glm::vec3 worldPos = glm::unProject(glm::vec3(winX, winY, winZ),
				view, projection, viewport);

			if (m_profilePointSelected == 1)
			{
				m_profilePoint1 = worldPos;
				m_profilePointSelected = 2;
				AfxMessageBox(_T("起点已选择，请点击选择剖面终点"));
			}
			else if (m_profilePointSelected == 2)
			{
				m_profilePoint2 = worldPos;
				m_selectingProfile = false;
				m_profilePointSelected = 0;
				CalculateProfile();
				m_showProfile = true; // 启用剖面图显示
				
			}
		}
		else
		{
			AfxMessageBox(_T("请点击地形表面"));
		}

		return; // 不处理其他鼠标逻辑
	}

	m_bLeftDown = TRUE; m_lastMousePos = point; SetCapture();
	CView::OnLButtonDown(nFlags, point);
}
void CTerrainView::OnRButtonDown(UINT nFlags, CPoint point) {
	m_bRightDown = TRUE; m_lastMousePos = point; SetCapture();
}
void CTerrainView::OnMButtonDown(UINT nFlags, CPoint point) {
	m_bMiddleDown = TRUE; m_lastMousePos = point; SetCapture();
}

void CTerrainView::OnLButtonUp(UINT nFlags, CPoint point) {
	m_bLeftDown = FALSE; ReleaseCapture();
}
void CTerrainView::OnRButtonUp(UINT nFlags, CPoint point) {
	m_bRightDown = FALSE; ReleaseCapture();
}
void CTerrainView::OnMButtonUp(UINT nFlags, CPoint point) {
	m_bMiddleDown = FALSE; ReleaseCapture();
}

void CTerrainView::OnMouseMove(UINT nFlags, CPoint point)
{
	if (m_bLeftDown) { // 平移
		float dx = (point.x - m_lastMousePos.x) * 0.1f;
		float dy = (point.y - m_lastMousePos.y) * 0.1f;
		glm::vec3 right = glm::normalize(glm::cross(m_cameraFront, m_cameraUp));
		m_cameraPos -= right * dx;
		m_cameraPos += m_cameraUp * dy;
		Invalidate(FALSE);
	}
	else if (m_bRightDown) { // 提升
		float dy = (point.y - m_lastMousePos.y) * 0.1f;
		m_cameraPos.y += dy;
		Invalidate(FALSE);
	}
	else if (m_bMiddleDown) { // 缩放
		float dy = (point.y - m_lastMousePos.y) * 0.1f;
		m_cameraPos += m_cameraFront * dy;
		Invalidate(FALSE);
	}
	m_lastMousePos = point;
}

BOOL CTerrainView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	m_cameraPos += m_cameraFront * (zDelta / 120.0f);
	Invalidate(FALSE);
	return TRUE;
}

GLuint CTerrainView::CompileShader(const char* source, GLenum type)
{
	GLuint shader = glCreateShader(type);
	glShaderSource(shader, 1, &source, NULL);
	glCompileShader(shader);

	GLint success;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
	if (!success) {
		char infoLog[512];
		glGetShaderInfoLog(shader, 512, NULL, infoLog);
		AfxMessageBox(CString(infoLog));
	}
	return shader;
}



CString CTerrainView::OpenFileDialog()
{
	CFileDialog dlg(TRUE, _T("tif"), _T("*.tif"),
		OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
		_T("DEM Files (*.tif)|*.tif||"));
	if (dlg.DoModal() == IDOK)
	{
		return dlg.GetPathName();
	}
	return _T("");
}

void CTerrainView::Onfile()
{
	// TODO: 在此添加命令处理程序代码
	// 清除当前场景
	ClearScene();

	// 打开文件对话框选择DEM文件
	CFileDialog dlgDEM(TRUE, _T("tif"), NULL, OFN_FILEMUSTEXIST | OFN_HIDEREADONLY,
		_T("DEM Files (*.tif;*.tiff;*.dem)|*.tif;*.tiff;*.dem|All Files (*.*)|*.*||"), this);

	if (dlgDEM.DoModal() != IDOK)
		return;

	CString demPath = dlgDEM.GetPathName();

	// 打开文件对话框选择影像文件
	CFileDialog dlgImage(TRUE, _T("tif"), NULL, OFN_FILEMUSTEXIST | OFN_HIDEREADONLY,
		_T("Image Files (*.tif;*.tiff;*.jpg;*.jpeg;*.png)|*.tif;*.tiff;*.jpg;*.jpeg;*.png|All Files (*.*)|*.*||"), this);

	if (dlgImage.DoModal() != IDOK)
		return;

	CString imagePath = dlgImage.GetPathName();

	// 加载DEM数据
	LoadDEM(CT2A(demPath));

	// 加载影像纹理
	LoadTIFFWithLibTiff(CT2A(imagePath));

	// 刷新视图
	Invalidate();
}

bool CTerrainView::ConvertGeoTransformToProjected(double* geoTransform) {
	// 初始化GDAL（确保已调用GDALAllRegister()）
	GDALAllRegister();

	// 定义源坐标系 (WGS-84 EPSG:4326)
	OGRSpatialReference oSourceSRS;
	oSourceSRS.SetWellKnownGeogCS("WGS84");
	oSourceSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

	// 定义目标坐标系 (CGCS2000 3度带39带 EPSG:4534)
	OGRSpatialReference oTargetSRS;
	// 使用importFromEPSG导入预定义坐标系
	if (oTargetSRS.importFromEPSG(4534) != OGRERR_NONE) {
		AfxMessageBox(_T("无法导入CGCS2000坐标系"));
		return false;
	}
	oTargetSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

	// 创建坐标变换对象
	OGRCoordinateTransformation* poCT =
		OGRCreateCoordinateTransformation(&oSourceSRS, &oTargetSRS);

	if (!poCT) {
		AfxMessageBox(_T("创建坐标变换对象失败"));
		return false;
	}

	// 转换左上角坐标
	double x1 = geoTransform[0]; // 原始经度
	double y1 = geoTransform[3]; // 原始纬度
	if (!poCT->Transform(1, &x1, &y1)) {
		AfxMessageBox(_T("左上角坐标转换失败"));
		OGRCoordinateTransformation::DestroyCT(poCT);
		return false;
	}

	// 转换右上角坐标 (x方向+1像素)
	double x2 = geoTransform[0] + geoTransform[1];
	double y2 = geoTransform[3];
	if (!poCT->Transform(1, &x2, &y2)) {
		AfxMessageBox(_T("右上角坐标转换失败"));
		OGRCoordinateTransformation::DestroyCT(poCT);
		return false;
	}

	// 转换左下角坐标 (y方向+1像素)
	double x3 = geoTransform[0];
	double y3 = geoTransform[3] + geoTransform[5];
	if (!poCT->Transform(1, &x3, &y3)) {
		AfxMessageBox(_T("左下角坐标转换失败"));
		OGRCoordinateTransformation::DestroyCT(poCT);
		return false;
	}

	// 计算新参数
	geoTransform[0] = x1;             // 左上角X (米)
	geoTransform[3] = y1;             // 左上角Y (米)
	geoTransform[1] = x2 - x1;        // 水平分辨率 (米/像素)
	geoTransform[5] = y3 - y1;        // 垂直分辨率 (米/像素)
	geoTransform[2] = 0.0;             // 旋转系数清零
	geoTransform[4] = 0.0;             // 旋转系数清零

	// 清理资源
	OGRCoordinateTransformation::DestroyCT(poCT);
	return true;
}

void CTerrainView::RegenerateTerrainVertices()
{
	m_vertices.clear();

	// 使用当前缩放比例重新生成顶点
	for (int y = 0; y < m_terrainHeight - 1; y++)
	{
		for (int x = 0; x < m_terrainWidth - 1; x++)
		{
			// 第一个三角形
			AddVertex(x, y, m_demData, m_scale);
			AddVertex(x + 1, y, m_demData, m_scale);
			AddVertex(x, y + 1, m_demData, m_scale);

			// 第二个三角形
			AddVertex(x + 1, y, m_demData, m_scale);
			AddVertex(x + 1, y + 1, m_demData, m_scale);
			AddVertex(x, y + 1, m_demData, m_scale);
		}
	}

	// 更新顶点缓冲区
	if (glIsBuffer(m_terrainVBO)) {
		glBindBuffer(GL_ARRAY_BUFFER, m_terrainVBO);
		glBufferData(GL_ARRAY_BUFFER,
			m_vertices.size() * sizeof(float),
			m_vertices.data(),
			GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
}

void CTerrainView::ClearScene()
{
	// 清除OpenGL资源
	if (m_terrainVAO) {
		glDeleteVertexArrays(1, &m_terrainVAO);
		m_terrainVAO = 0;
	}

	if (m_terrainVBO) {
		glDeleteBuffers(1, &m_terrainVBO);
		m_terrainVBO = 0;
	}

	if (m_textureID) {
		glDeleteTextures(1, &m_textureID);
		m_textureID = 0;
	}

	// 清空数据
	m_vertices.clear();
	m_terrainWidth = 0;
	m_terrainHeight = 0;

	// 重置相机位置
	m_cameraPos = glm::vec3(0.0f, 50.0f, 50.0f);
	m_cameraFront = glm::vec3(0.0f, -0.5f, -1.0f);
	m_cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

	// 刷新视图
	Invalidate();
}

// 初始化颜色表
void CTerrainView::InitializeColorRamp()
{
	// 定义高度值范围（归一化到0-1）
	m_colorRampValues = { 0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f };

	// 定义对应的颜色（蓝-绿-黄-橙-红）
	m_colorRampColors = {
		glm::vec3(0.0f, 0.0f, 0.8f),   // 深蓝色（最低）
		glm::vec3(0.0f, 0.6f, 0.0f),   // 深绿色
		glm::vec3(0.5f, 0.8f, 0.0f),   // 黄绿色
		glm::vec3(0.8f, 0.6f, 0.0f),   // 橙黄色
		glm::vec3(0.8f, 0.3f, 0.0f),   // 橙色
		glm::vec3(0.8f, 0.0f, 0.0f)    // 红色（最高）
	};
}
// 更新颜色表（可根据实际需求调整）
void CTerrainView::UpdateColorRamp()
{
	// 这里可以实现动态调整颜色表的逻辑
	// 例如根据DEM的最大最小高度值重新计算颜色映射
	ApplyColorRamp();
}

void CTerrainView::OnViewTexture()
{
	m_showTexture = !m_showTexture;
	if (m_showTexture) m_showColorRamp = false;  // 互斥
	Invalidate();  // 刷新视图
}

// 分层颜色显示切换
void CTerrainView::OnViewColorRamp()
{
	m_showColorRamp = !m_showColorRamp;
	if (m_showColorRamp) {
		m_showTexture = false;  // 互斥
		ApplyColorRamp();       // 应用颜色表
	}
	Invalidate();  // 刷新视图
}

// 更新纹理菜单项状态
void CTerrainView::OnUpdateViewTexture(CCmdUI* pCmdUI)
{
	pCmdUI->SetCheck(m_showTexture);
}

// 更新分层颜色菜单项状态
void CTerrainView::OnUpdateViewColorRamp(CCmdUI* pCmdUI)
{
	pCmdUI->SetCheck(m_showColorRamp);
}

void CTerrainView::SetTextureVisibility(bool visible)
{
	m_showTexture = visible;
	Invalidate(); // 重绘视图
}

void CTerrainView::ApplyColorRamp()
{
	if (m_demData.empty()) return;

	// 找到DEM的最小和最大高度
	float minHeight = *std::min_element(m_demData.begin(), m_demData.end());
	float maxHeight = *std::max_element(m_demData.begin(), m_demData.end());
	float range = maxHeight - minHeight;

	// 重新生成顶点数据，包含颜色信息
	m_vertices.clear();

	for (int y = 0; y < m_terrainHeight - 1; y++) {
		for (int x = 0; x < m_terrainWidth - 1; x++) {
			// 第一个三角形
			AddColoredVertex(x, y, m_demData, m_scale, minHeight, range);
			AddColoredVertex(x + 1, y, m_demData, m_scale, minHeight, range);
			AddColoredVertex(x, y + 1, m_demData, m_scale, minHeight, range);

			// 第二个三角形
			AddColoredVertex(x + 1, y, m_demData, m_scale, minHeight, range);
			AddColoredVertex(x + 1, y + 1, m_demData, m_scale, minHeight, range);
			AddColoredVertex(x, y + 1, m_demData, m_scale, minHeight, range);
		}
	}

	// 更新顶点缓冲区
	glBindBuffer(GL_ARRAY_BUFFER, m_terrainVBO);
	glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(float), m_vertices.data(), GL_STATIC_DRAW);
}

void CTerrainView::AddColoredVertex(int x, int y, const std::vector<float>& demData,
	float scale, float minHeight, float range)
{
	// 计算顶点位置和纹理坐标（与AddVertex相同）
	double geoX = m_demGeoTransform[0] + x * m_demGeoTransform[1] + y * m_demGeoTransform[2];
	double geoY = m_demGeoTransform[3] + x * m_demGeoTransform[4] + y * m_demGeoTransform[5];

	double offsetX = geoX - m_terrainCenter.x;
	double offsetY = geoY - m_terrainCenter.y;

	float xPos = offsetX * 0.01;
	float zPos = offsetY * 0.01;
	float yPos = demData[y * m_terrainWidth + x] * scale;

	// 计算纹理坐标
	double texPixelX = (geoX - m_textureGeoTransform[0]) / m_textureGeoTransform[1];
	double texPixelY = (geoY - m_textureGeoTransform[3]) / m_textureGeoTransform[5];
	float u = static_cast<float>(texPixelX / (m_texturePixelWidth - 1));
	float v = 1.0f - static_cast<float>(texPixelY / (m_texturePixelHeight - 1));
	u = std::clamp(u, 0.0f, 1.0f);
	v = std::clamp(v, 0.0f, 1.0f);

	// 计算归一化高度
	float height = demData[y * m_terrainWidth + x];
	float normalizedHeight = (height - minHeight) / range;

	// 根据高度值选择颜色
	glm::vec3 color(0.5f, 0.5f, 0.8f); // 默认颜色
	for (size_t i = 0; i < m_colorRampValues.size() - 1; ++i) {
		if (normalizedHeight >= m_colorRampValues[i] &&
			normalizedHeight < m_colorRampValues[i + 1]) {
			float t = (normalizedHeight - m_colorRampValues[i]) /
				(m_colorRampValues[i + 1] - m_colorRampValues[i]);
			color = glm::mix(m_colorRampColors[i], m_colorRampColors[i + 1], t);
			break;
		}
	}

	// 添加顶点数据：位置(3) + 纹理坐标(2) + 颜色(3)
	m_vertices.insert(m_vertices.end(), { xPos, yPos, zPos, u, v, color.r, color.g, color.b });
}




void CTerrainView::OnViewProfile()
{
	// TODO: 在此添加命令处理程序代码
	StartProfileSelection();
}

void CTerrainView::StartProfileSelection()
{
	m_selectingProfile = true;
	m_profilePointSelected = 1; // 从第一个点开始选择
	AfxMessageBox(_T("请在地形图上点击选择剖面起点"));
}

// 计算剖面数据
void CTerrainView::CalculateProfile()
{
	m_profileData.clear();

	// 计算两点之间的水平距离
	float dx = m_profilePoint2.x - m_profilePoint1.x;
	float dz = m_profilePoint2.z - m_profilePoint1.z;
	float distance = sqrt(dx * dx + dz * dz);

	// 采样点数（根据距离动态调整）
	int samples = static_cast<int>(distance * 10);
	if (samples < 2) samples = 2;

	// 计算采样点
	for (int i = 0; i <= samples; ++i)
	{
		float t = static_cast<float>(i) / samples;
		float x = m_profilePoint1.x + t * dx;
		float z = m_profilePoint1.z + t * dz;

		// 找到最近的地形顶点获取高度
		float y = 0.0f;
		float minDist = FLT_MAX;

		for (size_t j = 0; j < m_vertices.size(); j += 5)
		{
			float vx = m_vertices[j];
			float vz = m_vertices[j + 2];
			float dist = (x - vx) * (x - vx) + (z - vz) * (z - vz);

			if (dist < minDist)
			{
				minDist = dist;
				y = m_vertices[j + 1]; // 高度在y坐标
			}
		}

		m_profileData.push_back(glm::vec3(t * distance, y, 0.0f));
	}
}

void CTerrainView::DrawProfileInView(CDC* pDC) {
	CRect rectClient;
	GetClientRect(&rectClient);

	// 设定剖面图显示区域（左上角，宽度300，高度200）
	CRect profileRect(10, 10, 310, 210);

	// 1. 绘制背景
	pDC->FillSolidRect(profileRect, RGB(240, 240, 240));
	pDC->Draw3dRect(profileRect, RGB(100, 100, 100), RGB(200, 200, 200));

	// 2. 计算数据范围
	float minY = FLT_MAX, maxY = -FLT_MAX;
	for (const auto& point : m_profileData) {
		if (point.y < minY) minY = point.y;
		if (point.y > maxY) maxY = point.y;
	}
	float rangeY = maxY - minY;

	// 3. 绘制坐标轴
	pDC->MoveTo(profileRect.left + 30, profileRect.bottom - 30);
	pDC->LineTo(profileRect.right - 10, profileRect.bottom - 30); // X轴
	pDC->MoveTo(profileRect.left + 30, profileRect.top + 10);
	pDC->LineTo(profileRect.left + 30, profileRect.bottom - 30);  // Y轴

	// 4. 绘制剖面曲线
	CPen penProfile(PS_SOLID, 2, RGB(255, 0, 0));
	CPen* pOldPen = pDC->SelectObject(&penProfile);

	for (size_t i = 0; i < m_profileData.size(); ++i) {
		float x = static_cast<float>(i) / (m_profileData.size() - 1);
		float y = (m_profileData[i].y - minY) / rangeY;

		int posX = profileRect.left + 30 + static_cast<int>(x * (profileRect.Width() - 40));
		int posY = profileRect.bottom - 30 - static_cast<int>(y * (profileRect.Height() - 40));

		if (i == 0) {
			pDC->MoveTo(posX, posY);
		}
		else {
			pDC->LineTo(posX, posY);
		}
	}

	// 5. 恢复旧笔并释放资源
	pDC->SelectObject(pOldPen);
	penProfile.DeleteObject();

}