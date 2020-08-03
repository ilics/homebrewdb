//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include <iostream>
#include <fstream>
#include <valarray>

#include "marker_detector.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/core/eigen.hpp>

#include <Eigen/Eigen>
#include <ctime>


using namespace std;
using namespace cv;

Marker::Marker()
{
    id=-1;
    metric_size=-1;
    pose.setIdentity();
}

Marker::Marker(const Marker &M):std::vector<cv::Point2f>(M)
{
    id=M.id;
    metric_size=M.metric_size;
    pose.setIdentity();
}

Marker::Marker(const  std::vector<cv::Point2f> &corners,int _id):std::vector<cv::Point2f>(corners)
{
    id=_id;
    metric_size=-1;
    pose.setIdentity();
}

void Marker::draw(cv::Mat &in, cv::Scalar color, int lineWidth ,bool writeId)const
{
    if (size()!=4) return;
    line(in,(*this)[0],(*this)[1],color,lineWidth,cv::LINE_AA);
    line(in,(*this)[1],(*this)[2],color,lineWidth, cv::LINE_AA);
    line(in,(*this)[2],(*this)[3],color,lineWidth, cv::LINE_AA);
    line(in,(*this)[3],(*this)[0],color,lineWidth, cv::LINE_AA);
    rectangle(in,(*this)[0]- cv::Point2f(2,2),(*this)[0]+ cv::Point2f(2,2), cv::Scalar(0,0,255,255),lineWidth, cv::LINE_AA);
    rectangle(in,(*this)[1]- cv::Point2f(2,2),(*this)[1]+ cv::Point2f(2,2), cv::Scalar(0,255,0,255),lineWidth, cv::LINE_AA);
    rectangle(in,(*this)[2]- cv::Point2f(2,2),(*this)[2]+ cv::Point2f(2,2), cv::Scalar(255,0,0,255),lineWidth, cv::LINE_AA);
    if (writeId) {
        char cad[100];
        sprintf(cad,"id=%d",id);
        //determine the centroid
        Point cent(0,0);
        for (int i=0;i<4;i++)
        {
            cent.x+=(*this)[i].x;
            cent.y+=(*this)[i].y;
        }
        cent.x/=4.;
        cent.y/=4.;
        putText(in,cad, cent,FONT_HERSHEY_SIMPLEX, 0.5,  Scalar(255-color[0],255-color[1],255-color[2],255),2);
    }
}

void print(cv::Point3f p,string cad){
    cout<<cad<<" "<<p.x<<" "<<p.y<< " "<<p.z<<endl;
}

void Marker::calculateExtrinsics(float markerSizeMeters,cv::Mat  camMatrix,cv::Mat distCoeff)throw(cv::Exception)
{
    if (!isValid()) throw cv::Exception(9004,"!isValid(): invalid marker. It is not possible to calculate extrinsics","calculateExtrinsics",__FILE__,__LINE__);

    double halfSize=markerSizeMeters/2.;
    cv::Mat ObjPoints(4,3,CV_32FC1);
    ObjPoints.at<float>(1,0)=-halfSize;
    ObjPoints.at<float>(1,1)=halfSize;
    ObjPoints.at<float>(1,2)=0;
    ObjPoints.at<float>(2,0)=halfSize;
    ObjPoints.at<float>(2,1)=halfSize;
    ObjPoints.at<float>(2,2)=0;
    ObjPoints.at<float>(3,0)=halfSize;
    ObjPoints.at<float>(3,1)=-halfSize;
    ObjPoints.at<float>(3,2)=0;
    ObjPoints.at<float>(0,0)=-halfSize;
    ObjPoints.at<float>(0,1)=-halfSize;
    ObjPoints.at<float>(0,2)=0;

    cv::Mat ImagePoints(4,2,CV_32FC1);

    //Set image points from the marker
    for (int c=0;c<4;c++)
    {
        ImagePoints.at<float>(c,0)=((*this)[c].x);
        ImagePoints.at<float>(c,1)=((*this)[c].y);
    }

    cv::Mat raux,taux;
    cv::solvePnP(ObjPoints, ImagePoints, camMatrix, distCoeff,raux,taux);
    raux.convertTo(Rvec,CV_32F);
    taux.convertTo(Tvec ,CV_32F);

    Vector3f travec(Tvec.at<float>(0,0),Tvec.at<float>(1,0),Tvec.at<float>(2,0));
    Vector3f rotvec(Rvec.at<float>(0,0),Rvec.at<float>(1,0),Rvec.at<float>(2,0));
    float norm = rotvec.norm();
    AngleAxisf rot(norm,rotvec/norm);
    pose.translation() = travec;
    pose.linear() = rot.toRotationMatrix();
    metric_size=markerSizeMeters;
    //cout<<(*this)<<endl;

}


cv::Point2f Marker::getCenter()const
{
    cv::Point2f cent(0,0);
    for(size_t i=0;i<size();i++){
        cent.x+=(*this)[i].x;
        cent.y+=(*this)[i].y;
    }
    cent.x/=float(size());
    cent.y/=float(size());
    return cent;
}

float Marker::getArea()const
{
    assert(size()==4);
    //use the cross products
    cv::Point2f v01=(*this)[1]-(*this)[0];
    cv::Point2f v03=(*this)[3]-(*this)[0];
    float area1=fabs(v01.x*v03.y - v01.y*v03.x);
    cv::Point2f v21=(*this)[1]-(*this)[2];
    cv::Point2f v23=(*this)[3]-(*this)[2];
    float area2=fabs(v21.x*v23.y - v21.y*v23.x);
    return (area2+area1)/2.;


}

float Marker::getPerimeter()const
{
    assert(size()==4);
    float sum=0;
    for(int i=0;i<4;i++)
        sum+=norm( (*this)[i]-(*this)[(i+1)%4]);
    return sum;
}

Board::Board() {pose.setIdentity();}


Board::Board ( string filePath ) throw ( cv::Exception ) {
    readFromFile ( filePath );
    pose.setIdentity();

}

Board::Board ( const Board  &T ) : reference ( T.reference )
{ pose.setIdentity();}


Board & Board ::operator= ( const Board  &T ) {
    reference = T.reference;
    return *this;
    pose.setIdentity();
}

void Board::saveToFile ( string sfile ){

    cv::FileStorage fs ( sfile,cv::FileStorage::WRITE );
    fs<<"bc_nmarkers"<< ( int ) reference.size();
    fs<<"bc_markers"<<"[";
    for ( size_t i=0; i<reference.size(); i++ ) {
        fs << "{:" << "id" << reference.at(i).id ;

        fs<<"corners"<< "[:";
        for ( uint c=0; c<reference.at(i).size(); c++ )
            fs<<reference.at(i)[c];
        fs<<"]";
        fs <<  "}";
    }
    fs << "]";
}

void Board::readFromFile ( string sfile ) {

    cv::FileStorage fs ( sfile,cv::FileStorage::READ );
    int aux=0;
    //look for the nmarkers
    if ( fs["bc_nmarkers"].name() !="bc_nmarkers" )
    {
        cerr << "Board file in wrong format or missing!" << endl;
        return;
    }

    fs["bc_nmarkers"]>>aux;
    reference.resize ( aux );
    cv::FileNode markers=fs["bc_markers"];
    int i=0;
    for ( FileNodeIterator it = markers.begin(); it!=markers.end(); ++it,i++ ) {
        reference.at(i).id= ( *it ) ["id"];
        FileNode FnCorners= ( *it ) ["corners"];
        for ( FileNodeIterator itc = FnCorners.begin(); itc!=FnCorners.end(); ++itc ) {
            vector<float> coordinates3d;
            ( *itc ) >>coordinates3d;
            assert ( coordinates3d.size() ==3 );
            cv::Point3f point ( coordinates3d[0],coordinates3d[1],coordinates3d[2] );
            reference.at(i).push_back ( point );
        }
    }


}



int Board::getIndexOfMarkerId ( int id )
{

	for (size_t i = 0; i < reference.size(); i++)
	{
		if (reference.at(i).id == id) return i;
	}
    return -1;
}


const Marker3D& Board::getMarkerInfo ( int id ){
    for ( size_t i=0; i<reference.size(); i++ )
        if ( reference.at(i).id ==id ) return reference.at ( i );
    assert(0); // If here, marker not in board!!

}

void Board::getIdList ( std::vector< int >& ids, bool append ) const
{
    if ( !append ) ids.clear();
    for ( size_t i=0; i<reference.size(); i++ )
        ids.push_back ( reference.at ( i ).id );
}


SubPixelCorner::SubPixelCorner()
{
    _winSize=15;
    _apertureSize=3;
    _term.maxCount=10;
    _term.epsilon=0.1;
    _term.type=CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
    enable=true;
}

void SubPixelCorner::checkTerm()
{
    switch( _term.type)
    {
    case CV_TERMCRIT_ITER:
        _term.epsilon = 0.f;
        break;
    case CV_TERMCRIT_EPS:
        _term.maxCount=_term.COUNT;
        break;
    case CV_TERMCRIT_ITER | CV_TERMCRIT_EPS:
        break;
    default:
        _term.maxCount=_term.COUNT;
        _term.epsilon=0.1;
        _term.type=CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
        break;
    }

    eps = std::max( _term.epsilon ,0.0);
    eps=eps*eps;

    _max_iters= std::max( _term.maxCount, 1 );
    int max1=TermCriteria::MAX_ITER;
    _max_iters= std::min(_max_iters,max1);

}

double SubPixelCorner::pointDist(cv::Point2f estimate_corner,cv::Point2f curr_corner)
{
    double dist=((curr_corner.x-estimate_corner.x)*(curr_corner.x-estimate_corner.x))+
            ((curr_corner.y-estimate_corner.y)*(curr_corner.y-estimate_corner.y));
    return dist;
}


void SubPixelCorner::generateMask()
{

    double coeff = 1. / (_winSize*_winSize);
    float * maskX=(float *)calloc(1,(_winSize*sizeof(float)));
    float * maskY=(float *)calloc(1,(_winSize*sizeof(float)));
    mask.create (_winSize,_winSize,CV_32FC(1));
    /* calculate mask */
    for( int i = -_winSize/2, k = 0; i <= _winSize/2; i++, k++ )
        maskX[k] = (float)exp( -i * i * coeff );

    maskY = maskX;

    for( int i = 0; i < _winSize; i++ )
    {
        float * mask_ptr=mask.ptr <float>(i);
        for( int j = 0; j < _winSize; j++ )
            mask_ptr[j] = maskX[j] * maskY[i];
    }

}

void SubPixelCorner::RefineCorner(cv::Mat image,std::vector <cv::Point2f> &corners)
{

    if(enable==false)
        return;
    checkTerm();

    generateMask ();
    //loop over all the corner points
    for(uint k=0;k<corners.size ();k++)
    {
        cv::Point2f curr_corner;
        //initial estimate
        cv::Point2f estimate_corner=corners[k];

        if(estimate_corner.x<0 || estimate_corner.y<0 || estimate_corner.y >image.rows || estimate_corner.y > image.cols)
            continue;
        int iter=0;
        double dist=TermCriteria::EPS;
        //loop till termination criteria is met
        do
        {
            iter=iter+1;
            curr_corner=estimate_corner;

            Mat local;
            cv::getRectSubPix (image,Size(_winSize+2*(_apertureSize/2),_winSize+2*(_apertureSize/2)),curr_corner,local);

            cv::Mat Dx,Dy;
            //extracing image ROI about the corner point
            //Mat local=image(roi);
            //computing the gradients over the neighborhood about corner point
            cv::Sobel (local,Dx,CV_32FC(1),1,0,_apertureSize,1,0);
            cv::Sobel (local,Dy,CV_32FC(1),0,1,_apertureSize,1,0);

            //parameters requried for estimations
            double A=0,B=0,C=0,D=0,E=0,F=0;
            int lx=0,ly=0;
            for(int i=_apertureSize/2;i<=_winSize;i++)
            {

                float *dx_ptr=Dx.ptr <float>(i);
                float *dy_ptr=Dy.ptr <float>(i);
                ly=i-_winSize/2-_apertureSize/2;

                float * mask_ptr=mask.ptr <float>(ly+_winSize/2);

                for(int j=_apertureSize/2;j<=_winSize;j++)
                {

                    lx=j-_winSize/2-_apertureSize/2;
                    //cerr << lx+_winSize/2 << ":" ;
                    double val=mask_ptr[lx+_winSize/2];
                    double dxx=dx_ptr[j]*dx_ptr[j]*val;
                    double dyy=dy_ptr[j]*dy_ptr[j]*val;
                    double dxy=dx_ptr[j]*dy_ptr[j]*val;

                    A=A+dxx;
                    B=B+dxy;
                    E=E+dyy;
                    C=C+dxx*lx+dxy*ly;
                    F=F+dxy*lx+dyy*ly;

                }
            }

            //computing denominator
            double det=(A*E-B*B);
            if(fabs( det ) > DBL_EPSILON*DBL_EPSILON)
            {
                det=1.0/det;
                //translating back to original corner and adding new estimates
                estimate_corner.x=curr_corner.x+((C*E)-(B*F))*det;
                estimate_corner.y=curr_corner.y+((A*F)-(C*D))*det;
            }
            else
            {
                estimate_corner.x=curr_corner.x;
                estimate_corner.y=curr_corner.y;
            }

            dist=pointDist(estimate_corner,curr_corner);


        }while(iter<_max_iters && dist>eps);

        //double dist=pointDist(corners[k],estimate_corner);
        if(fabs(corners[k].x-estimate_corner.x) > _winSize || fabs(corners[k].y-estimate_corner.y)>_winSize)
        {
            estimate_corner.x=corners[k].x;
            estimate_corner.y=corners[k].y;
        }
        corners[k].x=estimate_corner.x;
        corners[k].y=estimate_corner.y;
        //cerr << "EEE" << corners[k].x <<":" << corners[k].y << endl;

    }

}

MarkerCode::MarkerCode(unsigned int n) {
    // resize bits vectors and initialize to 0
    for(unsigned int i=0; i<4; i++) {
        _bits[i].resize(n*n);
        for(unsigned int j=0; j<_bits[i].size(); j++) _bits[i][j]=0;
        _ids[i] = 0; // ids are also 0
    }
    _n = n;
}


MarkerCode::MarkerCode(const MarkerCode& MC)
{
    for(unsigned int i=0; i<4; i++) {
        _bits[i] = MC._bits[i];
        _ids[i] = MC._ids[i];
    }
    _n = MC._n;
}

unsigned int MarkerCode::selfDistance(unsigned int &minRot) {
    unsigned int res = _bits[0].size(); // init to n*n (max value)
    for(unsigned int i=1; i<4; i++) { // self distance is not calculated for rotation 0
        unsigned int hammdist = hammingDistance(_bits[0], _bits[i]);
        if(hammdist<res) {
            minRot = i;
            res = hammdist;
        }
    }
    return res;
}

/**
 */
unsigned int MarkerCode::distance(MarkerCode m, unsigned int &minRot) {
    unsigned int res = _bits[0].size(); // init to n*n (max value)
    for(unsigned int i=0; i<4; i++) {
        unsigned int hammdist = hammingDistance(_bits[0], m.getRotation(i));
        if(hammdist<res) {
            minRot = i;
            res = hammdist;
        }
    }
    return res;
}


void MarkerCode::fromString(std::string s)
{
    for(unsigned int i=0; i<s.length(); i++) {
        if(s[i]=='0') set(i, false);
        else set(i, true);
    }
}


std::string MarkerCode::toString()
{
    std::string s;
    s.resize(size());
    for(unsigned int i=0; i<size(); i++) {
        if(get(i)) s[i]='1';
        else s[i]='0';
    }
    return s;
}


unsigned int MarkerCode::hammingDistance(std::vector<bool> m1, std::vector<bool> m2) {
    unsigned int res=0;
    for(unsigned int i=0; i<m1.size(); i++)
        if(m1[i]!=m2[i]) res++;
    return res;
}


void MarkerCode::set(unsigned int pos, bool val) {
    // if not the same value
    if( get(pos) != val ) {
        for(unsigned int i=0; i<4; i++) { // calculate bit coordinates for each rotation
            unsigned int y=pos/n(), x=pos%n(); // if rotation 0, dont do anything
            // else calculate bit position in that rotation
            if(i==1) { unsigned int aux=y; y=x; x=n()-aux-1; }
            else if(i==2) { y=n()-y-1; x=n()-x-1; }
            else if(i==3) { unsigned int aux=y; y=n()-x-1; x=aux; }
            unsigned int rotPos = y*n()+x; // calculate position in the unidimensional string
            _bits[i][rotPos] = val; // modify value
            // update identifier in that rotation
            if(val==true) _ids[i] += (uint)pow(float(2),float(rotPos)); // if 1, add 2^pos
            else _ids[i] -= (uint)pow(float(2),float(rotPos)); // if 0, substract 2^pos
        }
    }
}



cv::Mat MarkerCode::getImg(unsigned int pixSize) {
    const unsigned int borderSize=1;
    unsigned int nrows = n()+2*borderSize;
    if(pixSize%nrows != 0) pixSize = pixSize + nrows - pixSize%nrows;
    unsigned int cellSize = pixSize / nrows;
    cv::Mat img(pixSize, pixSize, CV_8U, cv::Scalar::all(0)); // create black image (init image to 0s)
    // double for to go over all the cells
    for(unsigned int i=0; i<n(); i++) {
        for(unsigned int j=0; j<n(); j++) {
            if(_bits[0][i*n()+j]!=0) { // just draw if it is 1, since the image has been init to 0
                // double for to go over all the pixels in the cell
                for(unsigned int k=0; k<cellSize; k++) {
                    for(unsigned int l=0; l<cellSize; l++) {
                        img.at<uchar>((i+borderSize)*cellSize+k, (j+borderSize)*cellSize+l) = 255;
                    }
                }
            }
        }
    }
    return img;
}


bool Dictionary::fromFile(std::string filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    int nmarkers, markersize;

    // read number of markers
    fs["nmarkers"] >> nmarkers; // cardinal of D
    fs["markersize"] >> markersize; // n

    // read each marker info
    for (int i=0; i<nmarkers; i++) {
        std::string s;
        fs["marker_" + toStr(i)] >> s;
        MarkerCode m(markersize);
        m.fromString(s);
        push_back(m);
    }
    fs.release();

    return true;
}

bool Dictionary::toFile(std::string filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    // save number of markers
    fs << "nmarkers" << (int)size(); // cardinal of D
    fs << "markersize" << (int)( (*this)[0].n() ); // n
    // save each marker code
    for (unsigned int i=0; i<size(); i++) {
        std::string s =  ((*this)[i]).toString();
        fs << "marker_" + toStr(i) << s;
    }
    fs.release();
    return true;
}


unsigned int Dictionary::distance(MarkerCode m, unsigned int &minMarker, unsigned int &minRot) {
    unsigned int res = m.size();
    for(unsigned int i=0; i<size(); i++) {
        unsigned int minRotAux;
        unsigned int distance = (*this)[i].distance(m, minRotAux);
        if(distance<res) {
            minMarker = i;
            minRot = minRotAux;
            res=distance;
        }
    }
    return res;
}


uint Dictionary::minimumDistance()
{
    if(size()==0) return 0;
    unsigned int minDist = (*this)[0].size();
    // for each marker in D
    for(unsigned int i=0; i<size(); i++) {
        // calculate self distance of the marker
        minDist = std::min( minDist, (*this)[i].selfDistance() );

        // calculate distance to all the following markers
        for(unsigned int j=i+1; j<size(); j++)
            minDist = std::min( minDist, (*this)[i].distance((*this)[j]) );

    }
    return minDist;
}

void Dictionary::dumpMarkers()
{
    for (uint i=0;i < this->size();i++)
    {
        stringstream ss;
        ss << i ;
        cv::imwrite("marker_"+ss.str()+".png",(*this)[i].getImg(250));
    }
}





void BalancedBinaryTree::loadDictionary(Dictionary &D) {
    // create _orderD wich is a sorted version of D
    _orderD.clear();
    for(unsigned int i=0; i<D.size(); i++)
        _orderD.push_back(pair<uint,uint>(D[i].getId() ,i) );

    std::sort(_orderD.begin(), _orderD.end());

    // calculate the number of levels of the tree
    unsigned int levels=0;
    while( pow(float(2),float(levels)) <= _orderD.size() ) levels++;
    //       levels-=1; // only count full levels

    // auxiliar vector to know which elements are already in the tree
    std::vector<bool> visited;
    visited.resize(_orderD.size());
    for(unsigned int i=0; i<_orderD.size(); i++) visited[i]=false;

    // calculate position of the root element
    unsigned int rootIdx = _orderD.size()/2;
    _root = rootIdx;
    visited[rootIdx] = true; // mark it as visited

    // auxiliar vector to store the ids intervals (max and min) during the creation of the tree
    std::vector< std::pair<unsigned int, unsigned int> > intervals;
    // first, add the two intervals at each side of root element
    intervals.push_back( std::pair<unsigned int, unsigned int>(0,rootIdx) );
    intervals.push_back( std::pair<unsigned int, unsigned int>(rootIdx,_orderD.size()) );

    // init the tree
    _binaryTree.clear();
    _binaryTree.resize(_orderD.size());

    // add root information to the tree (make sure child do not coincide with self root for small sizes of D)
    if(!visited[(0+rootIdx)/2]) _binaryTree[rootIdx].first = (0+rootIdx)/2;
    else _binaryTree[rootIdx].first = -1;
    if(!visited[(rootIdx+_orderD.size())/2]) _binaryTree[rootIdx].second = (rootIdx+_orderD.size())/2;
    else _binaryTree[rootIdx].second = -1;

    // for each tree level
    for(unsigned int i=1; i<levels; i++) {
        unsigned int nintervals = intervals.size(); // count number of intervals and process them
        for(unsigned int j=0; j<nintervals; j++) {
            // store interval information and delete it
            unsigned int lowerBound, higherBound;
            lowerBound = intervals.back().first;
            higherBound = intervals.back().second;
            intervals.pop_back();

            // center of the interval
            unsigned int center = (higherBound + lowerBound)/2;

            // if center not visited, continue
            if(!visited[center])	visited[center]=true;
            else continue;

            // calculate centers of the child intervals
            unsigned int lowerChild = (lowerBound + center)/2;
            unsigned int higherChild = (center + higherBound)/2;

            // if not visited (lower child)
            if(!visited[lowerChild]) {
                intervals.insert( intervals.begin(), std::pair<unsigned int, unsigned int>(lowerBound, center) ); // add the interval to analyze later
                _binaryTree[center].first = lowerChild; // add as a child in the tree
            }
            else _binaryTree[center].first = -1; // if not, mark as no child

            // (higher child, same as lower child)
            if(!visited[higherChild]) {
                intervals.insert( intervals.begin(), std::pair<unsigned int, unsigned int>(center, higherBound) );
                _binaryTree[center].second = higherChild;
            }
            else _binaryTree[center].second = -1;

        }
    }

}


bool BalancedBinaryTree::findId(unsigned int id, unsigned int &orgPos) {
    int pos = _root; // first position is root
    while(pos!=-1) { // while having a valid position
        unsigned int posId = _orderD[pos].first; // calculate id of the node
        if(posId == id ) {
            orgPos = _orderD[pos].second;
            return true; // if is the desire id, return true
        }
        else if(posId < id) pos = _binaryTree[pos].second; // if desired id is higher, look in higher child
        else pos = _binaryTree[pos].first; // if it is lower, look in lower child
    }
    return false; // if nothing found, return false
}





MarkerDetector::MarkerDetector()
{
    _doErosion=false;
    _thresMethod=ADPT_THRES;
    _thresParam1=_thresParam2=7;
    _cornerMethod=SUBPIX;
    _markerWarpSize=70;
    _speed=0;
    pyrdown_level=0; // no image reduction
    _minSize=0.01;
    _maxSize=0.2;
    _repj_err_thres = 0.5;

    // Calibrated RGB!!!-values for my Carmine 1.09
//    camera = Mat::eye(3,3,CV_32F);
//    camera.at<float>(0,0) = 534.77f;
//    camera.at<float>(1,1) = 534.2f;
//    camera.at<float>(0,2) = 319.81;
//    camera.at<float>(1,2) = 242.2;


//    distortion = Mat::zeros(8,1,CV_32F);
//    distortion.at<float>(0,0) = 0.2457;
//    distortion.at<float>(1,0) = -0.88075;
//    distortion.at<float>(2,0) = 0.0;
//    distortion.at<float>(4,0) = 0.0;
//    distortion.at<float>(5,0) = 1.1291;

    distortion = Mat::zeros(8,1,CV_32F);
	/**distortion.at<float>(0, 0) = 0.0420;
	distortion.at<float>(1, 0) = -0.053407;
	distortion.at<float>(2, 0) = -0.000476;
	distortion.at<float>(3, 0) = -0.000941;
	distortion.at<float>(4, 0) = 0.006815;**/
	// distortion.at<float>(0, 0) = 0.049335712;
 //    distortion.at<float>(1,0) = -0.1877871;
 //    distortion.at<float>(2,0) = 0;
 //    distortion.at<float>(4,0) = 0;
 //    distortion.at<float>(5,0) = 0.1215039;

    _borderDistThres=0.01;//corners in a border of 1% of image  are ignored

	threads = 1;
	curr_thread = 0;

	num_detected = 0;
}

MarkerDetector::MarkerDetector(Mat camera, Mat distortion)
{
    _doErosion=false;
    _thresMethod=ADPT_THRES;
    _thresParam1=_thresParam2=7;
    _cornerMethod=LINES;
    _markerWarpSize=56;
    _speed=0;
    pyrdown_level=0; // no image reduction
    _minSize=0.01;
    _maxSize=0.2;
    _repj_err_thres = 1;

    camera.copyTo(this->camera);
    distortion.copyTo(this->distortion);

    _borderDistThres=0.01;//corners in a border of 1% of image  are ignored

	threads = 1;
	curr_thread = 0;

	num_detected = 0;
}


MarkerDetector::~MarkerDetector(){}


void MarkerDetector::setDesiredSpeed ( int val )
{
    if ( val<0 ) val=0;
    else if ( val>3 ) val=2;

    _speed=val;
    switch ( _speed )
    {

    case 0:
        _markerWarpSize=56;
        _cornerMethod=SUBPIX;
        _doErosion=true;
        break;

    case 1:
    case 2:
        _markerWarpSize=28;
        _cornerMethod=NONE;
        break;
    };
}

void MarkerDetector::init(Matrix3f camera)
{
    eigen2cv(camera,this->camera);
}


void MarkerDetector::detectMarkers (const cv::Mat &input,float markerSizeMeters) throw ( cv::Exception )
{

    //clear input data
    detectedMarkers.clear();
    if (D.empty()) return;
    if(input.empty()) return;

    if ( input.type() ==CV_8UC3 )   cv::cvtColor ( input,grey,CV_BGR2GRAY );
    else grey=input;

    cv::Mat imgToBeThresHolded=grey;
    double ThresParam1=_thresParam1,ThresParam2=_thresParam2;
    //Must the image be downsampled before continue pocessing?
    if ( pyrdown_level!=0 )
    {
        reduced=grey;
        for ( int i=0;i<pyrdown_level;i++ )
        {
            cv::Mat tmp;
            cv::pyrDown ( reduced,tmp );
            reduced=tmp;
        }
        int red_den=pow ( 2.0f,pyrdown_level );
        imgToBeThresHolded=reduced;
        ThresParam1/=float ( red_den );
        ThresParam2/=float ( red_den );
    }

    //Do threshold the image and detect contours
    thresHold ( _thresMethod,imgToBeThresHolded,thres,ThresParam1,ThresParam2 );
	//cv::imshow("thresh", thres);
	//cv::waitKey(0);
    //an erosion might be required to detect chessboard like boards
    if ( _doErosion )
    {
        erode ( thres,thres2,cv::Mat() );
        thres2.copyTo(thres); //vs thres=thres2;
    }
    //find all rectangles in the thresholdes image
    vector<MarkerCandidate > candidates;
    detectRectangles ( thres,candidates );


    //if the image has been downsampled, then calcualte the location of the corners in the original image
    if ( pyrdown_level!=0 )
    {
        float red_den=pow ( 2.0f,pyrdown_level );
        float offInc= ( ( pyrdown_level/2. )-0.5 );
        for (uint i=0;i<candidates.size();i++ ) {
            for ( int c=0;c<4;c++ )
            {
                candidates[i][c].x=candidates[i][c].x*red_den+offInc;
                candidates[i][c].y=candidates[i][c].y*red_den+offInc;
            }
            //do the same with the the contour points
            for (uint c=0;c<candidates[i].contour.size();c++ )
            {
                candidates[i].contour[c].x=candidates[i].contour[c].x*red_den+offInc;
                candidates[i].contour[c].y=candidates[i].contour[c].y*red_den+offInc;
            }
        }
    }

    //identify the markers
    vector<vector<Marker> >markers_omp(threads);
    vector<vector < vector<Point2f> > >candidates_omp(threads);
//#pragma omp parallel for
    for ( int i=0;i<candidates.size();i++ )
    {
        //Find projective homography
        Mat canonicalMarker;
        bool resW=false;
        resW = warp(grey,canonicalMarker,Size( _markerWarpSize,_markerWarpSize ),candidates[i]);
        if (resW) {
            int nRotations;
            int id= identifyMarker( canonicalMarker,nRotations );
            if ( id!=-1 )
            {
                if(_cornerMethod==LINES) refineCandidateLines( candidates[i]);
                markers_omp[curr_thread].push_back ( candidates[i] );
                markers_omp[curr_thread].back().id=id;
                //sort the points so that they are always in the same order no matter the camera orientation
                std::rotate ( markers_omp[curr_thread].back().begin(),
                              markers_omp[curr_thread].back().begin() +4-nRotations,markers_omp[curr_thread].back().end() );
            }
            else candidates_omp[curr_thread].push_back ( candidates[i] );
        }

    }

    //unify parallel data
    joinVectors(markers_omp,detectedMarkers,true);
    joinVectors(candidates_omp,_candidates,true);

    //refine the corner location if desired
    if ( detectedMarkers.size() >0 && _cornerMethod!=NONE && _cornerMethod!=LINES )
    {
        vector<Point2f> Corners;
        for ( uint i=0;i<detectedMarkers.size();i++ )
            for ( int c=0;c<4;c++ )
                Corners.push_back ( detectedMarkers[i][c] );

        if ( _cornerMethod==HARRIS )
        {
            SubPixelCorner Subp;
            Subp.RefineCorner(grey,Corners);
        }
        else if ( _cornerMethod==SUBPIX )
            cornerSubPix ( grey, Corners,Size(5,5),Size(-1,-1 ),TermCriteria (CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,3,0.05 ) );

        //copy back
        for (uint i=0;i<detectedMarkers.size();i++ )
            for ( int c=0;c<4;c++ ) detectedMarkers[i][c]=Corners[i*4+c];
    }

    //sort by id
    std::sort (detectedMarkers.begin(),detectedMarkers.end());
    // there might be still the case that a marker is detected twice because of
    // the double border indicated earlier, detect and remove these cases
    int borderDistThresX=_borderDistThres*float(input.cols);
    int borderDistThresY=_borderDistThres*float(input.rows);
    vector<bool> toRemove ( detectedMarkers.size(),false );
    for ( int i=0;i<int ( detectedMarkers.size() )-1;i++ )
    {
        if ( detectedMarkers[i].id==detectedMarkers[i+1].id && !toRemove[i+1] )
        {
            //deletes the one with smaller perimeter
            if (perimeter(detectedMarkers[i]) >perimeter(detectedMarkers[i+1]))
                toRemove[i+1]=true;
            else toRemove[i]=true;
        }
        //delete if any of the corners is too near image border
        for(size_t c=0;c<detectedMarkers[i].size();c++){
            if ( detectedMarkers[i][c].x<borderDistThresX ||
                 detectedMarkers[i][c].y<borderDistThresY ||
                 detectedMarkers[i][c].x>input.cols-borderDistThresX ||
                 detectedMarkers[i][c].y>input.rows-borderDistThresY ) toRemove[i]=true;

        }
    }

    //remove the invalid ones by setting the valid in the positions left by the invalids
    size_t indexValid=0;
    for (size_t i=0;i<toRemove.size();i++) {
        if (!toRemove[i]) {
            if (indexValid!=i) detectedMarkers[indexValid]=detectedMarkers[i];
            indexValid++;
        }
    }
    detectedMarkers.resize(indexValid);

    //detect the position of detected markers
    for (auto &m : detectedMarkers)
        m.calculateExtrinsics(markerSizeMeters,camera,distortion);

}

int MarkerDetector::identifyMarker(const Mat &in, int &nRotations)
{

    assert(in.rows==in.cols);
    cv::Mat grey;
    if ( in.type()==CV_8UC1) grey=in;
    else cv::cvtColor(in,grey,cv::COLOR_BGR2GRAY);
    //threshold image
    cv::threshold(grey, grey,125, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
    _swidth=grey.rows/_ncellsBorder;

    // obtain inner code
    MarkerCode candidate(_n);
    for (uint y=0;y<_n;y++)
        for (uint x=0;x<_n;x++)
        {
            int Xstart=(x+1)*(_swidth);
            int Ystart=(y+1)*(_swidth);
            cv::Mat square=grey(cv::Rect(Xstart,Ystart,_swidth,_swidth));
            int nZ=countNonZero(square);
            if (nZ> (_swidth*_swidth) /2)  candidate.set(y*_n+x, 1);
        }

    // search each marker id in the balanced binary tree
    unsigned int orgPos;
    for(unsigned int i=0; i<4; i++) {
        if(_binaryTree.findId( candidate.getId(i), orgPos )) {
            nRotations = i;
            return candidate.getId(i);
        }
    }


    // correct errors
    uint minMarker, minRot;
    if(D.distance(candidate, minMarker, minRot) <= _correctionDistance) {
        nRotations = minRot;
        return D[minMarker].getId();
    }

    return -1;

}


float MarkerDetector::detect(const cv::Mat &input,float markerSizeMeters) throw ( cv::Exception )
{

    detectMarkers(input,markerSizeMeters);
//cv::imshow( "thresholded", getThresholdedImage() ); cv::waitKey();
    if (board.reference.empty())
		return 0;

    board.detected.clear();
    std::cout << detectedMarkers.size() << " markers detected" << std::endl;
    num_detected = detectedMarkers.size();

    //find among detected markers these that belong to the board configuration
    for (Marker &m : detectedMarkers) {

        if ( board.getIndexOfMarkerId(m.id)!=-1 ) {
            board.detected.push_back (m);
            board.detected.back().metric_size=markerSizeMeters;
        }
	}

    if (board.detected.empty()) return 0;

    double scale=markerSizeMeters/cv::norm(board.reference[0][0]-board.reference[0][1]);

    // now, create the matrices for finding the extrinsics
    vector<cv::Point3f> objPoints;
    vector<cv::Point2f> imgPoints;
    for ( Marker &m : board.detected)
        for ( int p=0; p<4; p++ ) {
            imgPoints.push_back(m[p]);
            objPoints.push_back(board.getMarkerInfo(m.id)[p]*scale);
        }

    cv::Mat rvec,tvec;
    cv::solvePnP ( objPoints,imgPoints,camera,distortion,rvec,tvec );
    rvec.convertTo ( board.Rvec,CV_32FC1 );
    tvec.convertTo ( board.Tvec,CV_32FC1 );

    //now, do a refinement and remove points whose reprojection error is above a threshold, then repeat calculation with the rest
    vector<cv::Point2f> reproj;
    cv::projectPoints(objPoints,rvec,tvec,camera,distortion,reproj);
    vector<int> pointsThatPassTest;
    for (size_t i=0; i<reproj.size(); i++)
        if (cv::norm(reproj[i]-imgPoints[i])<_repj_err_thres )
            pointsThatPassTest.push_back(i);


    //copy these data to another vectors and repeat
    vector<cv::Point3f> objPoints_filtered;
    vector<cv::Point2f> imagePoints_filtered;
    for (size_t i=0; i<pointsThatPassTest.size(); i++) {
        objPoints_filtered.push_back(objPoints[pointsThatPassTest[i]]);
        imagePoints_filtered.push_back(imgPoints[pointsThatPassTest[i]]);
    }

    cv::solvePnP(objPoints,imgPoints,camera,distortion,rvec,tvec );
    rvec.convertTo(board.Rvec,CV_32FC1);
    tvec.convertTo(board.Tvec,CV_32FC1);
    cv::Mat rotMat;
    cv::Rodrigues(rvec,rotMat);

    Eigen::Map<Matrix3d> eigenR( &rotMat.at<double>(0) );

    board.pose.setIdentity();
    board.pose.linear() = eigenR.cast<float>();
    board.pose.translation() = -board.pose.linear()*Vector3f(board.Tvec.at<float>(0,0),board.Tvec.at<float>(1,0),board.Tvec.at<float>(2,0));

    float prob=float ( board.detected.size() ) /double ( board.reference.size() );

/*    // Estimate the board plane in the scene
    board.plane_point.setZero();
    Matrix3f M;
    M.setZero();
    for (cv::Point3f &p : objPoints_filtered)
    {
        Vector3f curr = board.pose*Vector3f(p.x,p.y,p.z);
        M += curr*curr.transpose();
        board.plane_point += curr;
    }
    board.plane_point /= objPoints_filtered.size();


    EigenSolver<Matrix3f> es(M);
    int i; es.eigenvalues().real().minCoeff(&i);
    board.plane_normal = es.eigenvectors().col(i).real();
    if (board.plane_normal(2) > 0) board.plane_normal *= -1;*/


    return prob;
}





void  MarkerDetector::detectRectangles ( const cv::Mat &thres,vector<std::vector<cv::Point2f> > &MarkerCandidates )
{
    vector<MarkerCandidate>  candidates;
    detectRectangles(thres,candidates);
    //create the output
    MarkerCandidates.resize(candidates.size());
    for (size_t i=0;i<MarkerCandidates.size();i++)
        MarkerCandidates[i]=candidates[i];
}

void MarkerDetector::detectRectangles(const cv::Mat &thresImg,vector<MarkerCandidate> & OutMarkerCanditates)
{
    vector<MarkerCandidate>  MarkerCandidates;
    //calcualte the min_max contour sizes
    uint minSize=_minSize*std::max(thresImg.cols,thresImg.rows)*4;
    uint maxSize=_maxSize*std::max(thresImg.cols,thresImg.rows)*4;
    std::vector<std::vector<cv::Point> > contours2;
    std::vector<cv::Vec4i> hierarchy2;

    thresImg.copyTo ( thres2 );
    cv::findContours ( thres2 , contours2, hierarchy2,CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
    vector<Point>  approxCurve;


    //for each contour, analyze if it is a paralelepiped likely to be the marker
    for ( unsigned int i=0;i<contours2.size();i++ )
    {
        //check it is a possible element by first checking is has enough points
        if ( minSize< contours2[i].size() &&contours2[i].size()<maxSize  )
        {
            //approximate to a polygon
            approxPolyDP (contours2[i],approxCurve,double(contours2[i].size())*0.05,true);
            //check that the polygon has 4 points and convex
            if ( approxCurve.size() ==4 && isContourConvex ( Mat ( approxCurve ) ))
            {


                // 	ensure that the distace between consecutive points is large enough
                float minDist=1e10;
                for ( int j=0;j<4;j++ )
                {
                    float d= std::sqrt ( ( float ) ( approxCurve[j].x-approxCurve[ ( j+1 ) %4].x ) * ( approxCurve[j].x-approxCurve[ ( j+1 ) %4].x ) +
                            ( approxCurve[j].y-approxCurve[ ( j+1 ) %4].y ) * ( approxCurve[j].y-approxCurve[ ( j+1 ) %4].y ) );
                    if ( d<minDist ) minDist=d;
                }
                //check that distance is not very small
                if ( minDist>10 )
                {
                    //add the points
                    // 	      cout<<"ADDED"<<endl;
                    MarkerCandidates.push_back(MarkerCandidate());
                    MarkerCandidates.back().idx=i;
                    for (int j=0;j<4;j++ )
                        MarkerCandidates.back().push_back(Point2f(approxCurve[j].x,approxCurve[j].y));
                }
            }
        }
    }


    //sort the points in anti-clockwise order
    valarray<bool> swapped(false,MarkerCandidates.size());//used later
    for ( unsigned int i=0;i<MarkerCandidates.size();i++ )
    {

        //trace a line between the first and second point.
        //if the thrid point is at the right side, then the points are anti-clockwise
        double dx1 = MarkerCandidates[i][1].x - MarkerCandidates[i][0].x;
        double dy1 =  MarkerCandidates[i][1].y - MarkerCandidates[i][0].y;
        double dx2 = MarkerCandidates[i][2].x - MarkerCandidates[i][0].x;
        double dy2 = MarkerCandidates[i][2].y - MarkerCandidates[i][0].y;
        double o = ( dx1*dy2 )- ( dy1*dx2 );

        if (o < 0.0 )		 //if the third point is in the left side, then sort in anti-clockwise order
        {
            swap ( MarkerCandidates[i][1],MarkerCandidates[i][3] );
            swapped[i]=true;
            //sort the contour points
            //  	    reverse(MarkerCanditates[i].contour.begin(),MarkerCanditates[i].contour.end());//????

        }
    }

    // remove these elements which corners are too close to each other
    //first detect candidates to be removed

    vector< vector<pair<int,int>  > > TooNearCandidates_omp(threads);
//#pragma omp parallel for
    for ( int i=0;i<MarkerCandidates.size();i++ )
    {
        // 	cout<<"Marker i="<<i<<MarkerCanditates[i]<<endl;
        //calculate the average distance of each corner to the nearest corner of the other marker candidate
        for ( unsigned int j=i+1;j<MarkerCandidates.size();j++ )
        {
            float dist=0;
            for ( int c=0;c<4;c++ )
                dist+= sqrt ( ( MarkerCandidates[i][c].x-MarkerCandidates[j][c].x ) *
                              ( MarkerCandidates[i][c].x-MarkerCandidates[j][c].x ) +
                              ( MarkerCandidates[i][c].y-MarkerCandidates[j][c].y ) *
                              ( MarkerCandidates[i][c].y-MarkerCandidates[j][c].y ) );
            dist/=4;
            //if distance is too small
            if ( dist< 10 )
                TooNearCandidates_omp[curr_thread].push_back ( pair<int,int> ( i,j ) );

        }
    }
    //join
    vector<pair<int,int>  > TooNearCandidates;
    joinVectors(  TooNearCandidates_omp,TooNearCandidates);
    //mark for removal the element of  the pair with smaller perimeter
    valarray<bool> toRemove ( false,MarkerCandidates.size() );
    for ( unsigned int i=0;i<TooNearCandidates.size();i++ )
    {
        if ( perimeter ( MarkerCandidates[TooNearCandidates[i].first ] ) >perimeter ( MarkerCandidates[ TooNearCandidates[i].second] ) )
            toRemove[TooNearCandidates[i].second]=true;
        else toRemove[TooNearCandidates[i].first]=true;
    }

    //remove the invalid ones
    //     removeElements ( MarkerCanditates,toRemove );
    //finally, assign to the remaining candidates the contour
    OutMarkerCanditates.reserve(MarkerCandidates.size());
    for (size_t i=0;i<MarkerCandidates.size();i++) {
        if (!toRemove[i]) {
            OutMarkerCanditates.push_back(MarkerCandidates[i]);
            OutMarkerCanditates.back().contour=contours2[ MarkerCandidates[i].idx];
            if (swapped[i] )//if the corners where swapped, it is required to reverse here the points so that they are in the same order
                reverse(OutMarkerCanditates.back().contour.begin(),OutMarkerCanditates.back().contour.end());//????
        }
    }

}

void MarkerDetector::thresHold (int method,const Mat &grey,Mat &out,double param1,double param2 )
{

    if (param1==-1) param1=_thresParam1;
    if (param2==-1) param2=_thresParam2;

    assert ( grey.type() ==CV_8UC1 );
    switch ( method )
    {
    case FIXED_THRES:
        cv::threshold ( grey, out, param1,255, CV_THRESH_BINARY_INV );
        break;
    case ADPT_THRES://currently, this is the best method
        //ensure that _thresParam1%2==1
        if ( param1<3 ) param1=3;
        else if ( ( ( int ) param1 ) %2 !=1 ) param1= ( int ) ( param1+1 );

        cv::adaptiveThreshold ( grey,out,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY_INV,param1,param2 );
        break;
    case CANNY:
        cv::Canny ( grey, out, 10, 220 );//this should be the best method, and generally it is. But sometimes problems
        break;
    }
}


bool MarkerDetector::warp ( Mat &in,Mat &out,Size size, vector<Point2f> points ) throw ( cv::Exception )
{

    assert ( points.size() ==4 );
    //obtain the perspective transform
    Point2f  pointsRes[4],pointsIn[4];
    for ( int i=0;i<4;i++ ) pointsIn[i]=points[i];
    pointsRes[0]= ( Point2f ( 0,0 ) );
    pointsRes[1]= Point2f ( size.width-1,0 );
    pointsRes[2]= Point2f ( size.width-1,size.height-1 );
    pointsRes[3]= Point2f ( 0,size.height-1 );
    Mat M=getPerspectiveTransform ( pointsIn,pointsRes );
    cv::warpPerspective ( in, out,  M, size,cv::INTER_NEAREST );
    return true;
}

bool MarkerDetector::isInto ( Mat &contour,vector<Point2f> &b )
{

    for ( unsigned int i=0;i<b.size();i++ )
        if ( pointPolygonTest ( contour,b[i],false ) >0 ) return true;
    return false;
}

int MarkerDetector:: perimeter ( vector<Point2f> &a )
{
    int sum=0;
    for ( unsigned int i=0;i<a.size();i++ )
    {
        int i2= ( i+1 ) %a.size();
        sum+= sqrt ( ( a[i].x-a[i2].x ) * ( a[i].x-a[i2].x ) + ( a[i].y-a[i2].y ) * ( a[i].y-a[i2].y ) ) ;
    }
    return sum;
}

void MarkerDetector::refineCandidateLines(MarkerDetector::MarkerCandidate& candidate)
{
    // search corners on the contour vector
    vector<unsigned int> cornerIndex;
    cornerIndex.resize(4);
    for(unsigned int j=0; j<candidate.contour.size(); j++)
        for(unsigned int k=0; k<4; k++)
            if(candidate.contour[j].x==candidate[k].x && candidate.contour[j].y==candidate[k].y)
                cornerIndex[k] = j;


    // contour pixel in inverse order or not?
    bool inverse;
    if( (cornerIndex[1] > cornerIndex[0]) && (cornerIndex[2]>cornerIndex[1] || cornerIndex[2]<cornerIndex[0]) )
        inverse = false;
    else if( cornerIndex[2]>cornerIndex[1] && cornerIndex[2]<cornerIndex[0] )
        inverse = false;
    else inverse = true;


    // get pixel vector for each line of the marker
    int inc = 1;
    if(inverse) inc = -1;

    // undistort contour
    vector<Point2f> contour2f;
    for(unsigned int i=0; i<candidate.contour.size(); i++)
        contour2f.push_back( cv::Point2f(candidate.contour[i].x, candidate.contour[i].y) );
    cv::undistortPoints(contour2f, contour2f,camera, distortion , cv::Mat(), camera);


    vector<std::vector<cv::Point2f> > contourLines;
    contourLines.resize(4);
    for(unsigned int l=0; l<4; l++) {
        for(int j=(int)cornerIndex[l]; j!=(int)cornerIndex[(l+1)%4]; j+=inc) {
            if(j==(int)candidate.contour.size() && !inverse) j=0;
            else if(j==0 && inverse) j=candidate.contour.size()-1;
            contourLines[l].push_back(contour2f[j]);
            if(j==(int)cornerIndex[(l+1)%4]) break; // this has to be added because of the previous ifs
        }

    }

    // interpolate marker lines
    vector<Point3f> lines;
    lines.resize(4);
    for(unsigned int j=0; j<lines.size(); j++) interpolate2Dline(contourLines[j], lines[j]);

    // get cross points of lines
    vector<Point2f> crossPoints;
    crossPoints.resize(4);
    for(unsigned int i=0; i<4; i++)
        crossPoints[i] = getCrossPoint( lines[(i-1)%4], lines[i] );

    // distort corners again
    distortPoints(crossPoints, crossPoints, camera, distortion );

    // reassing points
    for(unsigned int j=0; j<4; j++)
        candidate[j] = crossPoints[j];
}


void MarkerDetector::interpolate2Dline( const std::vector< Point2f >& inPoints, Point3f& outLine)
{

    float minX, maxX, minY, maxY;
    minX = maxX = inPoints[0].x;
    minY = maxY = inPoints[0].y;
    for(unsigned int i=1; i<inPoints.size(); i++)  {
        if(inPoints[i].x < minX) minX = inPoints[i].x;
        if(inPoints[i].x > maxX) maxX = inPoints[i].x;
        if(inPoints[i].y < minY) minY = inPoints[i].y;
        if(inPoints[i].y > maxY) maxY = inPoints[i].y;
    }

    // create matrices of equation system
    Mat A(inPoints.size(),2,CV_32FC1, Scalar(0));
    Mat B(inPoints.size(),1,CV_32FC1, Scalar(0));
    Mat X;

    if( maxX-minX > maxY-minY ) {
        // Ax + C = y
        for (uint i=0; i<inPoints.size(); i++)
        {
            A.at<float>(i, 0) = inPoints[i].x;
            A.at<float>(i, 1) = 1.;
            B.at<float>(i, 0) = inPoints[i].y;
        }

        // solve system
        solve(A,B,X, DECOMP_SVD);
        // return Ax + By + C
        outLine = Point3f(X.at<float>(0,0), -1., X.at<float>(1,0));
    }
    else {
        // By + C = x
        for (uint i=0; i<inPoints.size(); i++)
        {
            A.at<float>(i, 0) = inPoints[i].y;
            A.at<float>(i, 1) = 1.;
            B.at<float>(i, 0) = inPoints[i].x;
        }

        // solve system
        solve(A,B,X, DECOMP_SVD);
        // return Ax + By + C
        outLine = Point3f(-1., X.at<float>(0,0), X.at<float>(1,0));
    }

}

/**
 */
Point2f MarkerDetector::getCrossPoint(const cv::Point3f& line1, const cv::Point3f& line2)
{

    // create matrices of equation system
    Mat A(2,2,CV_32FC1, Scalar(0));
    Mat B(2,1,CV_32FC1, Scalar(0));
    Mat X;

    A.at<float>(0, 0) = line1.x;
    A.at<float>(0, 1) = line1.y;
    B.at<float>(0, 0) = -line1.z;

    A.at<float>(1, 0) = line2.x;
    A.at<float>(1, 1) = line2.y;
    B.at<float>(1, 0) = -line2.z;

    // solve system
    solve(A,B,X, DECOMP_SVD);
    return Point2f(X.at<float>(0,0), X.at<float>(1,0));

}



void MarkerDetector::distortPoints(vector<cv::Point2f> in, vector<cv::Point2f> &out, const Mat& camMatrix, const Mat& distCoeff)
{
    cv::Mat Rvec = cv::Mat(3,1,CV_32FC1, cv::Scalar::all(0));
    cv::Mat Tvec = Rvec.clone();
    // calculate 3d points and then reproject, so opencv makes the distortion internally
    vector<cv::Point3f> cornersPoints3d;
    for(unsigned int i=0; i<in.size(); i++)
        cornersPoints3d.push_back( cv::Point3f(
                                       (in[i].x-camMatrix.at<float>(0,2))/camMatrix.at<float>(0,0), 	//x
                                       (in[i].y-camMatrix.at<float>(1,2))/camMatrix.at<float>(1,1), 	//y
                                       1 ) );								//z
    cv::projectPoints(cornersPoints3d, Rvec, Tvec, camMatrix, distCoeff, out);
}


void MarkerDetector::draw ( Mat out,const vector<Marker> &markers )
{
    for ( unsigned int i=0;i<markers.size();i++ )
    {
        cv::line ( out,markers[i][0],markers[i][1],cvScalar ( 255,0,0 ),2,CV_AA );
        cv::line ( out,markers[i][1],markers[i][2],cvScalar ( 255,0,0 ),2,CV_AA );
        cv::line ( out,markers[i][2],markers[i][3],cvScalar ( 255,0,0 ),2,CV_AA );
        cv::line ( out,markers[i][3],markers[i][0],cvScalar ( 255,0,0 ),2,CV_AA );
    }
}


void MarkerDetector::dumpBoardImage(string filename, uint pixelpermarker)
{

    if(D.empty()) {
        std::cerr << "Error: No dictionary loaded" << std::endl;
        return;
    }

    if(board.reference.empty()) {
        std::cerr << "Error: No board loaded" << std::endl;
        return;
    }

    unsigned int MarkerSize = pixelpermarker;
    unsigned int MarkerDistance = MarkerSize/5;

    int sizeY=board.h*MarkerSize+(board.h-1)*MarkerDistance;
    int sizeX=board.w*MarkerSize+(board.w-1)*MarkerDistance;

    cv::Mat tableImage(sizeY,sizeX,CV_8UC1);
    tableImage.setTo(cv::Scalar(255));
    int idp=0;
    for (int y=0;y<board.h;y++)
        for (int x=0;x<board.w;x++,idp++) {
            cv::Mat subrect(tableImage,cv::Rect( x*(MarkerDistance+MarkerSize),y*(MarkerDistance+MarkerSize),MarkerSize,MarkerSize));
            cv::Mat marker=D[idp].getImg(MarkerSize);
            marker.copyTo(subrect);

        }
    cv::imwrite(filename, tableImage); // save output image

}

bool MarkerDetector::createBoard(string boardfile, int w, int h)
{
    if(D.empty()) {
        std::cerr << "Error: No dictionary loaded" << std::endl;
        return false;
    }

    board = Board();
    board.w = w;
    board.h = h;

    unsigned int MarkerSize = (D[0].n()+2)*20;
    unsigned int MarkerDistance = MarkerSize/5;

    int sizeY=board.h*MarkerSize+(board.h-1)*MarkerDistance;
    int sizeX=board.w*MarkerSize+(board.w-1)*MarkerDistance;
    //find the center so that the ref system is in it
    float centerX=sizeX*0.5f;
    float centerY=sizeY*0.5f;

    int idp=0;
    for (int y=0;y<board.h;y++)
        for (int x=0;x<board.w;x++,idp++) {

            // add to board configuration
            Marker3D MI;
            MI.resize(4);
            MI.id = D[idp].getId();
            MI[0].x = x*(MarkerDistance+MarkerSize) - centerX;
            MI[0].y = y*(MarkerDistance+MarkerSize) - centerY;
            MI[1].x = x*(MarkerDistance+MarkerSize)+MarkerSize - centerX;
            MI[1].y = y*(MarkerDistance+MarkerSize) - centerY;
            MI[2].x = x*(MarkerDistance+MarkerSize)+MarkerSize - centerX;
            MI[2].y = y*(MarkerDistance+MarkerSize)+MarkerSize - centerY;
            MI[3].x = x*(MarkerDistance+MarkerSize) - centerX;
            MI[3].y = y*(MarkerDistance+MarkerSize)+MarkerSize - centerY;
            for(int i=0; i<4; i++)
            {
                MI[0].y *= -1;   // y negative -> z points up
                MI[i].z = 0;
            }
            board.reference.push_back(MI);
        }

    board.saveToFile(boardfile); // save board configuration
    dumpBoardImage(boardfile+".png",MarkerSize);

    return true;
}


bool MarkerDetector::loadDictionary(string filename)
{
    if(!D.fromFile(filename)) {
        cerr<<"Could not open dictionary"<<endl;
        return false;
    }

    if(D.empty()) {
        cerr<<"Invalid dictionary!"<<endl;
        return false;
    }

    _n = D[0].n();
    _ncellsBorder = (D[0].n()+2);
    _correctionDistance = (uint)floor( (D.minimumDistance()-1)/2.0f ); //maximun correction distance
    _binaryTree.loadDictionary(D);
    return true;

}

bool MarkerDetector::loadBoard(string filename)
{
    board.readFromFile(filename);
    return true;
}

bool MarkerDetector::createDictionary(string filename, uint dictSize, uint n)
{
    typedef std::vector<bool> Word;

    class WordsLikeHood : public vector<double>
    {
        vector<Word> words;
        vector<unsigned int> chosen;
        double totalPosibilities; //(2^n)^n - (2^n - 1)^n

    public:

        WordsLikeHood(unsigned int wordSize)
        {
            resize( pow(float(2), float(wordSize)) );
            words.resize( size() );
            chosen.resize( size() );
            for(unsigned int i=0; i<size(); i++) {
                words[i] = wordFromNumber(i, wordSize);
                chosen[i] = 0;
            }
            srand(time(NULL));
            totalPosibilities = 1;
            update();
        }

        Word sampleWord()
        {
            int randInt = rand();
            double randDouble = (double)randInt/(double)RAND_MAX;
            double acc = 0;
            for(unsigned int i=0; i<size(); i++) {
                acc += (*this)[i];
                if(acc >= randDouble) return words[i];
            }
            return Word();
        }

        void incrementWord(Word w)
        {
            assert( words.size()>0 && w.size()==words[0].size() );
            unsigned int id = wordToNumber(w);
            chosen[id]++;
            totalPosibilities++;
        }

        void decrementWord(Word w)
        {
            assert( words.size()>0 && w.size()==words[0].size() );
            unsigned int id = wordToNumber(w);
            if(chosen[id]!=0) chosen[id]--;
            totalPosibilities--;
        }

        void update()
        {
            double total = 0.0;
            for(uint i=0; i<size(); i++) {
                (*this)[i] = ( 1.- (chosen[i])/(double)totalPosibilities  ) *
                        (wordTransitions(words[i])+1)/(double)(words[0].size());
                total+=(*this)[i];
            }

            for(uint i=0; i<size(); i++) {
                (*this)[i] /= total;
            }

        }

        uint wordToNumber(Word w)
        {
            unsigned int id=0;
            for(uint i=0; i<w.size() && i<sizeof(uint)*8; i++)
                id += pow(2.0f,float(i))*w[i];
            return id;
        }
        uint wordTransitions(Word w)
        {
            unsigned int count = 0;
            for(unsigned int i=0; i<w.size()-1; i++) {
                if(w[i]!=w[i+1]) count ++;
            }
            return count;
        }
        Word wordFromNumber(unsigned int n, unsigned int bits)
        {
            if(pow(float(2),float(bits)) < n ) {
                cerr << "Not enough number of bits to codify id " << n << endl;
            }

            Word res;
            res.resize(bits);
            for(unsigned int i=0; i<bits; i++) {
                if(n & (uint)pow(2.0f,float(i))) res[i] = 1;
                else res[i] = 0;
            }
            return res;
        }

    };

    D = Dictionary();

    unsigned int tau = 2*( ( 4*( (n*n)/4 ) )/3 );
    cout << "Tau: " << tau << endl;

    WordsLikeHood P(n);

    vector<Word> wordsInM;
    wordsInM.resize(n);

    unsigned int countUnproductive=0;
    while(D.size() < dictSize) {
        MarkerCode candidate( n );
        for(unsigned int i=0; i<n; i++) {
            P.update();
            wordsInM[i] = P.sampleWord();
            for(unsigned int j=0; j<n; j++) candidate.set(i*n+j, wordsInM[i][j]);
            P.incrementWord(wordsInM[i]);
        }

        if( D.distance(candidate)>=tau && candidate.selfDistance()>=tau) {
            D.push_back(candidate);
            cout << "Accepted Marker " << D.size() << "/" << dictSize << endl;
            countUnproductive=0;
        }
        else { // decrease words previously increased because they are not accepted!
            for(unsigned int i=0; i<n; i++) P.decrementWord(wordsInM[i]);
            countUnproductive++;
        }
        if(countUnproductive==5000) {
            tau--;
            countUnproductive=0;
            cout << "Reducing Tau to: " << tau << endl;
            if(tau==0) {
                std::cerr << "Error: Tau=0. Small marker size for too high number of markers. Stop" << std::endl;
                return false;
            }
        }
    }

    D.dumpMarkers();
    D.toFile(filename);
    return true;

}


void MarkerDetector::draw3dAxis(Mat &Image,Marker &m)
{

    float size=m.metric_size*3;
    Mat objectPoints (4,3,CV_32FC1);
    objectPoints.at<float>(0,0)=0;
    objectPoints.at<float>(0,1)=0;
    objectPoints.at<float>(0,2)=0;
    objectPoints.at<float>(1,0)=size;
    objectPoints.at<float>(1,1)=0;
    objectPoints.at<float>(1,2)=0;
    objectPoints.at<float>(2,0)=0;
    objectPoints.at<float>(2,1)=size;
    objectPoints.at<float>(2,2)=0;
    objectPoints.at<float>(3,0)=0;
    objectPoints.at<float>(3,1)=0;
    objectPoints.at<float>(3,2)=size;

    vector<Point2f> imagePoints;
    cv::projectPoints( objectPoints, m.Rvec,m.Tvec,camera, distortion,imagePoints);
    //draw lines of different colours
    cv::line(Image,imagePoints[0],imagePoints[1],Scalar(0,0,255,255),1,CV_AA);
    cv::line(Image,imagePoints[0],imagePoints[2],Scalar(0,255,0,255),1,CV_AA);
    cv::line(Image,imagePoints[0],imagePoints[3],Scalar(255,0,0,255),1,CV_AA);
    putText(Image,"x", imagePoints[1],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255,255),2);
    putText(Image,"y", imagePoints[2],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0,255),2);
    putText(Image,"z", imagePoints[3],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0,255),2);
}


void MarkerDetector::draw3dCube(Mat &Image,Marker &m)
{
    Mat objectPoints (8,3,CV_32FC1);
    double halfSize=m.metric_size/2;


    objectPoints.at<float>(0,0)=-halfSize;
    objectPoints.at<float>(0,1)=-halfSize;
    objectPoints.at<float>(0,2)=0;
    objectPoints.at<float>(1,0)=halfSize;
    objectPoints.at<float>(1,1)=-halfSize;
    objectPoints.at<float>(1,2)=0;
    objectPoints.at<float>(2,0)=halfSize;
    objectPoints.at<float>(2,1)=halfSize;
    objectPoints.at<float>(2,2)=0;
    objectPoints.at<float>(3,0)=-halfSize;
    objectPoints.at<float>(3,1)=halfSize;
    objectPoints.at<float>(3,2)=0;

    objectPoints.at<float>(4,0)=-halfSize;
    objectPoints.at<float>(4,1)=-halfSize;
    objectPoints.at<float>(4,2)=m.metric_size;
    objectPoints.at<float>(5,0)=halfSize;
    objectPoints.at<float>(5,1)=-halfSize;
    objectPoints.at<float>(5,2)=m.metric_size;
    objectPoints.at<float>(6,0)=halfSize;
    objectPoints.at<float>(6,1)=halfSize;
    objectPoints.at<float>(6,2)=m.metric_size;
    objectPoints.at<float>(7,0)=-halfSize;
    objectPoints.at<float>(7,1)=halfSize;
    objectPoints.at<float>(7,2)=m.metric_size;


    vector<Point2f> imagePoints;
    projectPoints( objectPoints, m.Rvec,m.Tvec,camera, distortion, imagePoints);
    //draw lines of different colours
    for (int i=0;i<4;i++)
        cv::line(Image,imagePoints[i],imagePoints[(i+1)%4],Scalar(0,0,255,255),1,CV_AA);

    for (int i=0;i<4;i++)
        cv::line(Image,imagePoints[i+4],imagePoints[4+(i+1)%4],Scalar(0,0,255,255),1,CV_AA);

    for (int i=0;i<4;i++)
        cv::line(Image,imagePoints[i],imagePoints[i+4],Scalar(0,0,255,255),1,CV_AA);

}

void MarkerDetector::drawBoardAxis(Mat &Image)
{
    float ssize = board.detected[0].metric_size;
    Mat objPts (4,3,CV_32FC1);
    objPts.at<float>(0,0)=0;objPts.at<float>(0,1)=0;objPts.at<float>(0,2)=0;
    objPts.at<float>(1,0)=2*ssize;objPts.at<float>(1,1)=0;objPts.at<float>(1,2)=0;
    objPts.at<float>(2,0)=0;objPts.at<float>(2,1)=2*ssize;objPts.at<float>(2,2)=0;
    objPts.at<float>(3,0)=0;objPts.at<float>(3,1)=0;objPts.at<float>(3,2)=2*ssize;

    vector<Point2f> imagePoints;
    projectPoints( objPts, board.Rvec,board.Tvec,camera,distortion, imagePoints);
    //draw lines of different colours
    cv::line(Image,imagePoints[0],imagePoints[1],Scalar(0,0,255,255),2,CV_AA);
    cv::line(Image,imagePoints[0],imagePoints[2],Scalar(0,255,0,255),2,CV_AA);
    cv::line(Image,imagePoints[0],imagePoints[3],Scalar(255,0,0,255),2,CV_AA);

    putText(Image,"X", imagePoints[1],FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255,255),2);
    putText(Image,"Y", imagePoints[2],FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0,255),2);
    putText(Image,"Z", imagePoints[3],FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,0,255),2);
}


void MarkerDetector::drawBoardCube(Mat &Image)
{

    float cubeSize=board.detected[0].metric_size;
    float txz=-cubeSize/2;

    vector<Vec3f> objPts;

    objPts.push_back(Vec3f(txz,txz,0));
    objPts.push_back(Vec3f(txz+cubeSize,txz,0));
    objPts.push_back(Vec3f(txz+cubeSize,txz+cubeSize,0));
    objPts.push_back(Vec3f(txz,txz+cubeSize,0));
    objPts.push_back(Vec3f(txz,txz,cubeSize));
    objPts.push_back(Vec3f(txz+cubeSize,txz,cubeSize));
    objPts.push_back(Vec3f(txz+cubeSize,txz+cubeSize,cubeSize));
    objPts.push_back(Vec3f(txz,txz+cubeSize,cubeSize));

    vector<Point2f> imagePoints;
    projectPoints( objPts,board.Rvec,board.Tvec, camera, distortion, imagePoints);
    //draw lines of different colours
    for(int i=0;i<4;i++)
        cv::line(Image,imagePoints[i],imagePoints[(i+1)%4],Scalar(255,0,255,255),2,CV_AA);

    for(int i=0;i<4;i++)
        cv::line(Image,imagePoints[i+4],imagePoints[4+(i+1)%4],Scalar(255,0,255,255),2,CV_AA);

    for(int i=0;i<4;i++)
        cv::line(Image,imagePoints[i],imagePoints[i+4],Scalar(0,0,255,255),2,CV_AA);
}