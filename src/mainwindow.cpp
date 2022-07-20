#include "mainwindow.h"

mainwindow::mainwindow()
{
    StyleWindow();
    connect(open,SIGNAL(clicked()),this,SLOT(lccpSeg()));
};

void mainwindow::StyleWindow()
{
	open = new QPushButton();
    open->setText(QString::fromLocal8Bit("update pcd"));
	labelVoxel = new QLabel();
    labelVoxel->setText("voxel_resolution");
	labelSeed = new QLabel();
    labelSeed->setText("seed_resolution");
	labelColor = new QLabel();
    labelColor->setText("color_importance");
	verLayout = new QGridLayout();
	lineVoxel = new QLineEdit();
	lineSeed = new QLineEdit();
	lineColor = new QLineEdit();
    verLayout->addWidget(labelVoxel,0,0,1,1);
    verLayout->addWidget(lineVoxel,0,1,1,1);
    verLayout->addWidget(labelSeed,1,0,1,1);
    verLayout->addWidget(lineSeed,1,1,1,1);
    verLayout->addWidget(labelColor,2,0,1,1);
    verLayout->addWidget(lineColor,2,1,1,1);

	Box = new QGroupBox();
    Box->setLayout(verLayout);
	ver = new QVBoxLayout(),
    ver->addWidget(Box);
    ver->addWidget(open);
	rightW = new QWidget();
    rightW->setLayout(ver);

	hor = new QHBoxLayout();
	qvtk = new QVTKWidget();
    hor->addWidget(qvtk);
    hor->addWidget(rightW);
    hor->setStretch(0,5);
    hor->setStretch(1,1);

	totalW = new QWidget();
    totalW->setLayout(hor);
    this->setCentralWidget(totalW);
}

mainwindow::~mainwindow(){};

void mainwindow::lccpSeg()
{   
    clock_t startTime,endTime;
    startTime = clock();
    if(lineVoxel->text()!=NULL && 
        lineSeed->text()!=NULL){
        voxel_resolution = this->lineVoxel->text().toFloat();
        seed_resolution = this->lineSeed->text().toFloat();
	    color_importance = this->lineColor->text().toFloat();
    }
    else{
        voxel_resolution = 0.0075f;
	    seed_resolution = 0.3f;
	    color_importance = 0.0f;
    }
    //输入点云
	pcl::PointCloud<PointT>::Ptr input_cloud_ptr(new pcl::PointCloud<PointT>);
	pcl::PCLPointCloud2 input_pointcloud2;
	if (pcl::io::loadPCDFile("milk_cartoon_all_small_clorox.pcd", input_pointcloud2))
	{
		PCL_ERROR("ERROR: Could not read input point cloud ");
		return ;
	}
	pcl::fromPCLPointCloud2(input_pointcloud2, *input_cloud_ptr);
	PCL_INFO("Done making cloud\n");
    
    float spatial_importance = 1.0f;
	float normal_importance = 4.0f;
	bool use_single_cam_transform = false;
	bool use_supervoxel_refinement = false;

	unsigned int k_factor = 0;
 
	//voxel_resolution is the resolution (in meters) of voxels used、seed_resolution is the average size (in meters) of resulting supervoxels  
	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	super.setUseSingleCameraTransform(use_single_cam_transform);
	super.setInputCloud(input_cloud_ptr);
	//Set the importance of color for supervoxels. 
	super.setColorImportance(color_importance);
	//Set the importance of spatial distance for supervoxels.
	super.setSpatialImportance(spatial_importance);
	//Set the importance of scalar normal product for supervoxels. 
	super.setNormalImportance(normal_importance);
	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
 
	PCL_INFO("Extracting supervoxels\n");
	super.extract(supervoxel_clusters);
 
	PCL_INFO("Getting supervoxel adjacency\n");
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);
	pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud(supervoxel_clusters);
 
	//LCCP分割
	float concavity_tolerance_threshold = 20;
	float smoothness_threshold = 0.1;
	uint32_t min_segment_size = 0;
	bool use_extended_convexity = false;
	bool use_sanity_criterion = false;
	PCL_INFO("Starting Segmentation\n");
	pcl::LCCPSegmentation<PointT> lccp;
	lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
	lccp.setKFactor(k_factor);
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	lccp.setMinSegmentSize(min_segment_size);
	lccp.segment();
	
	PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");
 
	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
	pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
	lccp.relabelCloud(*lccp_labeled_cloud);
	SuperVoxelAdjacencyList sv_adjacency_list;
	lccp.getSVAdjacencyList(sv_adjacency_list);

    // Configure Visualizer
    pcl::visualization::PCLVisualizer viewer = pcl::visualization::PCLVisualizer ("3D Viewer",false);
    viewer.addPointCloud (lccp_labeled_cloud, "Segmented point cloud");

	renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow = viewer.getRenderWindow();
    qvtk->SetRenderWindow(renderWindow);
    viewer.setupInteractor(qvtk->GetInteractor(),qvtk->GetRenderWindow());

    qvtk->update();
    qvtk->GetRenderWindow()->Render();
    endTime = clock();
    cout<<"The run time is :" << (double)(endTime-startTime)/CLOCKS_PER_SEC <<"s"<<endl;
}

