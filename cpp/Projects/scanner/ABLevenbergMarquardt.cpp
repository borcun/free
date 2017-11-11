#include "ABLevenbergMarquardt.hpp"

// default constructor
ABLevenbergMarquardt::ABLevenbergMarquardt()
{

}

// destructor
ABLevenbergMarquardt::~ABLevenbergMarquardt()
{
	// release matrices
	cvReleaseMat(&JMatFM);
	cvReleaseMat(&errMatFM);
	cvReleaseMat(&JtMat);
	cvReleaseMat(&JtJMat);
	cvReleaseMat(&epsMat);
	cvReleaseMat(&JtEpsMat);
	cvReleaseMat(&currentMatF);
	cvReleaseMat(&deltaMat);
	cvReleaseMat(&deltaMatT);
	cvReleaseMat(&lambdaDeltaMat);
}

// function that run LM algorithm and optimize fundamental matrix
// Source : http://www.ics.forth.gr/~lourakis/levmar/levmar.pdf
CvMat *ABLevenbergMarquardt::run(CvMat *_JMat, CvMat *_errMat, CvMat *matF, const vector<CvPoint2D64f> leftPoints, const vector<CvPoint2D64f> rightPoints, 
	const CvTermCriteria criteria)
{
	// check situations
	if(_errMat->cols != 1) {
		cerr << "The column of Error Matrix must be 1" << endl;
		return NULL;
	}
	
	if(_JMat->rows != _errMat->rows) {
		cerr << "The column of Error Matrix must equals column of Jacobian Matrix" << endl;
		return NULL;
	}

	// set iteration and epsilon values
	int iters = criteria.max_iter;
	const double err = (double)criteria.epsilon;
	// allocate memory for Jacobian matrix, error matrix and copy parameters to them
	JMatFM = cvCreateMat(_JMat->rows, _JMat->cols, CV_64FC1);
	errMatFM = cvCreateMat(_errMat->rows, _errMat->cols, CV_64FC1);
	cvCopy(_JMat, JMatFM);
	cvCopy(_errMat, errMatFM);
	// create other matrices
	// Jacobian and epsilon matrices
	JtMat = cvCreateMat(JMatFM->cols, JMatFM->rows, CV_64FC1);
	JtJMat = cvCreateMat(JMatFM->cols, JMatFM->cols, CV_64FC1);
	epsMat = cvCreateMat(JMatFM->rows, 1, CV_64FC1);
	JtEpsMat = cvCreateMat(JMatFM->cols, 1, CV_64FC1);
	// fundamental matrix variables
	currentMatF = cvCreateMat(matF->rows, matF->cols, CV_64FC1);
	// delta, its transpose and lambda . delta matrices
	deltaMat = cvCreateMat(JMatFM->cols, 1, CV_64FC1);
	deltaMatT = cvCreateMat(1, JMatFM->cols, CV_64FC1);
	lambdaDeltaMat = cvCreateMat(JMatFM->cols, 1, CV_64FC1);
	double lambda = 0.0;
	double ro = 1.0;
	double lambdaIncCoef = 2.0;
	// divide number and new epsilon matrix
	CvMat *newEpsMat = cvCreateMat(leftPoints.size(), 1, CV_64FC1);
	CvMat *newDivide = cvCreateMat(1, 1, CV_64FC1);

	// calculate JtEps
	cvTranspose(JMatFM, JtMat);
	// calculate JtJ matrix
	cvMulTransposed(JMatFM, JtJMat, 1);
	// copy errMatFM to epsMat
	cvCopy(errMatFM, epsMat);
	// calculate JtMat
	cvMatMul(JtMat, epsMat, JtEpsMat);
	// set lambda
	lambda = pow((double)10,8) * getMaxDiagonalElement(JtJMat);
    
    // check if infinity norm of JtEps equal or smaller than err
    // if statement is true, break loop
    if(getInfinityNorm(JtEpsMat) <= err) {
        // release matrices
        cvReleaseMat(&JtMat);
        cvReleaseMat(&JtJMat);
        cvReleaseMat(&JtEpsMat);
        cvReleaseMat(&epsMat);
        cvReleaseMat(&currentMatF);
        cvReleaseMat(&deltaMat);
        cvReleaseMat(&deltaMatT);
        cvReleaseMat(&lambdaDeltaMat);
        cvReleaseMat(&newEpsMat);
        cvReleaseMat(&newDivide);

        return matF;
    }
    
	// go on with decreasing iteration count
	while(--iters) {
		// calculate Delta from (JtJ + lambda . I) . delta = JtEps
		for(int i=0 ; i < JtJMat->rows ; ++i) {
			CV_MAT_ELEM(*JtJMat, double, i, i) += lambda;
		}

		// solve (JtJ + lambda . I) . Delta = JtEps
		cvSolve(JtJMat, JtEpsMat, deltaMat);
		
		// compare norms of deltaMat and MatF
		if(cvNorm(deltaMat) <= LM_THRESHOLD * cvNorm(matF)) {
			// release matrices
			cvReleaseMat(&JtMat);
			cvReleaseMat(&JtJMat);
			cvReleaseMat(&JtEpsMat);
			cvReleaseMat(&epsMat);
			cvReleaseMat(&currentMatF);
			cvReleaseMat(&deltaMat);
			cvReleaseMat(&deltaMatT);
			cvReleaseMat(&lambdaDeltaMat);
			cvReleaseMat(&newEpsMat);
			cvReleaseMat(&newDivide);
			
			return matF;
		}
		else {
			// add deltaMat to prevMatF and find new matF
			cvAdd(matF, deltaMat, currentMatF);
			// calculate new error
			cvCopy(getEpsMatrix(leftPoints, rightPoints, convert9x1to3x3(currentMatF)), newEpsMat);
			// find transpose of deltaMat
			cvTranspose(deltaMat, deltaMatT);

			// find lambda * deltaMat
			for(int i = 0 ; i < lambdaDeltaMat->rows ; ++i) {
				for(int j = 0 ; j < lambdaDeltaMat->cols ; ++j) {
					CV_MAT_ELEM(*lambdaDeltaMat, double, i, j) = CV_MAT_ELEM(*deltaMat, double, i, j) * lambda;
				}
			}

			// find lambdaDeltaMat + JtEpsMat
			cvAdd(lambdaDeltaMat, JtEpsMat, lambdaDeltaMat);
			cvMatMul(deltaMatT, lambdaDeltaMat, newDivide);

			// set ro if newDivide is not infinite
			// Source : http://www.johndcook.com/IEEE_exceptions_in_cpp.html
			if(CV_MAT_ELEM(*newDivide, double, 0, 0) <= DBL_MAX && CV_MAT_ELEM(*newDivide, double, 0, 0) >= -DBL_MAX)
				ro = (pow(cvNorm(epsMat), 2) - pow(cvNorm(newEpsMat), 2)) / CV_MAT_ELEM(*newDivide, double, 0, 0);
			else
				ro = 0.0;

			// update some values
			if(ro > 0) {
				// copy currentMatF to matF
				cvCopy(currentMatF, matF);
				// calculate new epsMat and JtEpsMat
				cvCopy(newEpsMat, epsMat);
				// update Jacobian matrix
				createJacobianMatrix(leftPoints, rightPoints, convert9x1to3x3(matF));
				// calculate JtEps
				cvTranspose(JMatFM, JtMat);
				// calculate JtJ matrix
				cvMulTransposed(JMatFM, JtJMat, 1);
				// calculate epsMat
				cvCopy(errMatFM, epsMat);
				// calculate JtEpsMat
				cvMatMul(JtMat, epsMat, JtEpsMat);

				// check if infinity norm of JtEps equal or smaller than err
				// if statement is true, break loop
				if(getInfinityNorm(JtEpsMat) <= err) {
					// release matrices
					cvReleaseMat(&JtMat);
					cvReleaseMat(&JtJMat);
					cvReleaseMat(&JtEpsMat);
					cvReleaseMat(&epsMat);
					cvReleaseMat(&currentMatF);
					cvReleaseMat(&deltaMat);
					cvReleaseMat(&deltaMatT);
					cvReleaseMat(&lambdaDeltaMat);
					cvReleaseMat(&newEpsMat);
					cvReleaseMat(&newDivide);

					return matF;
				}

				// update lambda 
				lambda *= MAX(1/3, 1 - (double)pow((2*ro - 1), 3));
				lambdaIncCoef = 2;
			}
			else {
				lambda *= lambdaIncCoef;
				lambdaIncCoef *= 2;

				// if lambda becomes overflow, break loop and exit from function
				if(lambda >= DBL_MAX && lambda <= -DBL_MAX) {
					// release matrices
					cvReleaseMat(&JtMat);
					cvReleaseMat(&JtJMat);
					cvReleaseMat(&JtEpsMat);
					cvReleaseMat(&epsMat);
					cvReleaseMat(&currentMatF);
					cvReleaseMat(&deltaMat);
					cvReleaseMat(&deltaMatT);
					cvReleaseMat(&lambdaDeltaMat);
					cvReleaseMat(&newEpsMat);
					cvReleaseMat(&newDivide);

					return matF;
				}
			}
		} // end of else
	} // end of while(--iters)

	// release matrices
	cvReleaseMat(&JtMat);
	cvReleaseMat(&JtJMat);
	cvReleaseMat(&JtEpsMat);
	cvReleaseMat(&epsMat);
	cvReleaseMat(&currentMatF);
	cvReleaseMat(&deltaMat);
	cvReleaseMat(&deltaMatT);
	cvReleaseMat(&lambdaDeltaMat);
	cvReleaseMat(&newEpsMat);
	cvReleaseMat(&newDivide);

	return matF;
}

// function that creates Jacobian matrix for fundamental matrix
void ABLevenbergMarquardt::createJacobianMatrix(const vector<CvPoint2D64f> leftPoints, const vector<CvPoint2D64f> rightPoints, const CvMat *funMat)
{
	/************************************************************/
	/*     F.x = | A B C |T     |	FT.x' = | A' B' C' |T		*/
	/*     A = f0x + f1y + f3   |	A' = f0x' + f3y' + f6		*/
	/*     B = f3x + f4y + f5	|	B' = f1x' + f4y' + f7		*/
	/*     C = f6x + f7y + f8	|	C' = f2x' + f5y' + f8		*/
	/************************************************************/
	/*     x'T.F.x = D = (f0xx' + f1yx' + f2x') +				*/
	/*				     (f3xy' + f4yy' + f5y') +				*/
	/*				     (f6x + f7y + f8)						*/
	/*															*/
	/*            M = A2 + B2 + A'2 + B'2						*/
	/************************************************************/
	/* Sampson Error: g = (x'T . F . x) / A2 + B2 + A'2 + B'2   */
	/************************************************************/
	/*		 J = dg = (x'T . F . x)' / (A2 + B2 + A'2 + B'2)'   */
	/************************************************************/

	// specify row and column sizes of Jacobian Matrix
	const int row = (int)leftPoints.size();
	const int col = (int)funMat->rows * (int)funMat->cols;
	register double D;
	register double M;
	register double A, Aprime, B, Bprime;
	// derivative variables these are derivation of g by f values
	register double gf0, gf1, gf2, gf3, gf4, gf5, gf6, gf7, gf8;
	// allocate memory for Jacobian and error matrices
	JMatFM = cvCreateMat(row, col, CV_64FC1);
	errMatFM = cvCreateMat(row, 1, CV_64FC1);

	// fill Jacobian matrix
	for(int i=0 ; i < row ; ++i) {
		// set A,B, Aprime and Bprime
		A = CV_MAT_ELEM(*funMat, double, 0, 0) * leftPoints.at(i).x + CV_MAT_ELEM(*funMat, double, 0, 1) * leftPoints.at(i).y + CV_MAT_ELEM(*funMat, double, 0, 2);
		B = CV_MAT_ELEM(*funMat, double, 1, 0) * leftPoints.at(i).x + CV_MAT_ELEM(*funMat, double, 1, 1) * leftPoints.at(i).y + CV_MAT_ELEM(*funMat, double, 1, 2);
		Aprime = CV_MAT_ELEM(*funMat, double, 0, 0) * rightPoints.at(i).x + CV_MAT_ELEM(*funMat, double, 1, 0) * rightPoints.at(i).y + CV_MAT_ELEM(*funMat, double, 2, 0);
		Bprime = CV_MAT_ELEM(*funMat, double, 0, 1) * rightPoints.at(i).x + CV_MAT_ELEM(*funMat, double, 1, 1) * rightPoints.at(i).y + CV_MAT_ELEM(*funMat, double, 2, 1);
		// set D
		D = CV_MAT_ELEM(*funMat, double, 0, 0) * leftPoints.at(i).x * rightPoints.at(i).x + 
			CV_MAT_ELEM(*funMat, double, 0, 1) * leftPoints.at(i).y * rightPoints.at(i).x + 
			CV_MAT_ELEM(*funMat, double, 0, 2) * rightPoints.at(i).x + 
			CV_MAT_ELEM(*funMat, double, 1, 0) * leftPoints.at(i).x * rightPoints.at(i).y + 
			CV_MAT_ELEM(*funMat, double, 1, 1) * leftPoints.at(i).y * rightPoints.at(i).y + 
			CV_MAT_ELEM(*funMat, double, 1, 2) * rightPoints.at(i).y + 
			CV_MAT_ELEM(*funMat, double, 2, 0) * leftPoints.at(i).x + 
			CV_MAT_ELEM(*funMat, double, 2, 1) * leftPoints.at(i).y +
			CV_MAT_ELEM(*funMat, double, 2, 2);

		// set M
		M = pow(A, 2) + pow(B, 2) + pow(Aprime, 2) + pow(Bprime, 2);
		// set gfX variables
		gf0 = (leftPoints.at(i).x * rightPoints.at(i).x / M) + ((2 * D * (A * leftPoints.at(i).x + Aprime * rightPoints.at(i).x)) / pow(M, 2));
		gf1 = (leftPoints.at(i).y * rightPoints.at(i).x / M) + ((2 * D * (A * leftPoints.at(i).y + Bprime * rightPoints.at(i).x)) / pow(M, 2));
		gf2 = (rightPoints.at(i).x / M) + ((2 * D * (A)) / pow(M, 2));
		gf3 = (leftPoints.at(i).x * rightPoints.at(i).y / M) + ((2 * D * (B * leftPoints.at(i).x + Aprime * rightPoints.at(i).y)) / pow(M, 2));
		gf4 = (leftPoints.at(i).y * rightPoints.at(i).y / M) + ((2 * D * (B * leftPoints.at(i).y + Bprime * rightPoints.at(i).y)) / pow(M, 2));
		gf5 = (rightPoints.at(i).y / M) + ((2 * D * (B)) / pow(M, 2));
		gf6 = (leftPoints.at(i).x / M) + ((2 * D * (Aprime)) / pow(M, 2));
		gf7 = (leftPoints.at(i).y / M) + ((2 * D * (Bprime)) / pow(M, 2));
		gf8 = 1 / M; // + ((2 * D * (0)) / pow(M, 2));

		// fill a row of Jacobian matrix
		CV_MAT_ELEM(*JMatFM, double, i, 0) = gf0; 
		CV_MAT_ELEM(*JMatFM, double, i, 1) = gf1;
		CV_MAT_ELEM(*JMatFM, double, i, 2) = gf2;
		CV_MAT_ELEM(*JMatFM, double, i, 3) = gf3;
		CV_MAT_ELEM(*JMatFM, double, i, 4) = gf4;
		CV_MAT_ELEM(*JMatFM, double, i, 5) = gf5;
		CV_MAT_ELEM(*JMatFM, double, i, 6) = gf6;
		CV_MAT_ELEM(*JMatFM, double, i, 7) = gf7;
		CV_MAT_ELEM(*JMatFM, double, i, 8) = gf8;
	} // end of for

	// set error matrix
	cvCopy(getEpsMatrix(leftPoints, rightPoints, funMat), errMatFM);

	return;
}

// function that gets new epsilon matrix
CvMat *ABLevenbergMarquardt::getEpsMatrix(const vector<CvPoint2D64f> leftPoints, const vector<CvPoint2D64f> rightPoints, const CvMat *F) {
	CvMat *newErr = cvCreateMat((int)leftPoints.size(), 1, CV_64FC1);
	CvMat *X1  = cvCreateMat(3, 1, CV_64FC1);
	CvMat *X2t = cvCreateMat(1, 3, CV_64FC1);
	CvMat *X2tFX1 = cvCreateMat(1, 1, CV_64FC1);

	// set error matrix
	for(int i=0 ; i < (int)leftPoints.size() ; ++i) {
		// create X2 point
		CV_MAT_ELEM(*X2t, double, 0, 0) = rightPoints.at(i).x;
		CV_MAT_ELEM(*X2t, double, 0, 1) = rightPoints.at(i).y;
		CV_MAT_ELEM(*X2t, double, 0, 2) = 1.0;
		// create X1 point
		CV_MAT_ELEM(*X1, double, 0, 0) = leftPoints.at(i).x;
		CV_MAT_ELEM(*X1, double, 1, 0) = leftPoints.at(i).y;
		CV_MAT_ELEM(*X1, double, 2, 0) = 1.0;
		// find result of X2t . F . X1
		cvMatMul(X2t, F, X2t);
		cvMatMul(X2t, X1, X2tFX1);

		CV_MAT_ELEM(*newErr, double, i, 0) = 0 - CV_MAT_ELEM(*X2tFX1, double, 0, 0);
	}

	// release matrices
	cvReleaseMat(&X1);
	cvReleaseMat(&X2t);
	cvReleaseMat(&X2tFX1);

	return newErr;
}

// function that gets Jacobian matrix
CvMat *ABLevenbergMarquardt::getJacobianMatrix() const 
{
	return JMatFM; 
}

// function that gets error matrix
CvMat *ABLevenbergMarquardt::getErrorMatrix() const 
{ 
	return errMatFM; 
}

// function that gets infinity norm
double ABLevenbergMarquardt::getInfinityNorm(const CvMat *m) {
	register double infinity_norm = fabs(CV_MAT_ELEM(*m, double, 0, 0));

	for(int i=0 ; i < m->rows ; ++i) {
		for(int j=0 ; j < m->cols ; ++j) {
			if(infinity_norm < fabs(CV_MAT_ELEM(*m, double, i, j)))  
                infinity_norm = fabs(CV_MAT_ELEM(*m, double, i, j));
		}
	}

	return infinity_norm;
} // end of getInfinityNorm

// function that gets maximum diagonal element
double ABLevenbergMarquardt::getMaxDiagonalElement(const CvMat *m) {
	if(m->rows != m->cols) {
		cerr << "Matrix must be square" << endl;
		return 0.0;
	}

	register double max_diagonal = CV_MAT_ELEM(*m, double, 0, 0);

	for(int i=1 ; i < m->rows ; ++i) {
		max_diagonal = max_diagonal < CV_MAT_ELEM(*m, double, i, i) ? CV_MAT_ELEM(*m, double, i, i) : max_diagonal;
	}

	return max_diagonal;
} // end of getMaxDiagonalElement

// function that converts 9x1 row matrix to 3x3 square matrix
CvMat *ABLevenbergMarquardt::convert9x1to3x3(const CvMat *row_matrix) {
	CvMat *mat = cvCreateMat(3, 3, CV_64FC1);

	// convert optimized 9x1 row matrix to 3x3 square matrix
	for(int i=0 ; i < 3 ; ++i) {
		for(int j=0 ; j < 3 ; ++j) {
			CV_MAT_ELEM(*mat, double, i, j) = CV_MAT_ELEM(*row_matrix, double, i*3+j, 0);
		}
	}

	return mat;
}


//CvMat *ABLevenbergMarquardt::runLMForEM(CvMat *_JMat, CvMat *_errMat, CvMat *matE, const vector<CvPoint3D64f> leftPoints, const vector<CvPoint3D64f> rightPoints, 
//	const CvTermCriteria criteria)
//{
//	// check situations
//	if(_errMat->cols != 1) {
//		cerr << "The column of Error Matrix must be 1" << endl;
//		return NULL;
//	}
//
//	if(_JMat->rows != _errMat->rows) {
//		cerr << "The column of Error Matrix must equals column of Jacobian Matrix" << endl;
//		return NULL;
//	}
//
//	// set iteration and epsilon values
//	int iters = criteria.max_iter;
//	const double err = (double)criteria.epsilon;
//	// allocate memory for Jacobian matrix, error matrix and copy parameters to them
//	JMatEM = cvCreateMat(_JMat->rows, _JMat->cols, CV_64FC1);
//	errMatEM = cvCreateMat(_errMat->rows, _errMat->cols, CV_64FC1);
//	cvCopy(_JMat, JMatEM);
//	cvCopy(_errMat, errMatEM);
//	// create other matrices
//	// Jacobian and epsilon matrices
//	JtMat = cvCreateMat(JMatEM->cols, JMatEM->rows, CV_64FC1);
//	JtJMat = cvCreateMat(JMatEM->cols, JMatEM->cols, CV_64FC1);
//	epsMat = cvCreateMat(JMatEM->rows, 1, CV_64FC1);
//	JtEpsMat = cvCreateMat(JMatEM->cols, 1, CV_64FC1);
//	// essential matrix variables
//	currentMatE = cvCreateMat(matE->rows, matE->cols, CV_64FC1);
//	// delta, its transpose and lambda . delta matrices
//	deltaMat = cvCreateMat(JMatEM->cols, 1, CV_64FC1);
//	deltaMatT = cvCreateMat(1, JMatEM->cols, CV_64FC1);
//	lambdaDeltaMat = cvCreateMat(JMatEM->cols, 1, CV_64FC1);
//	double lambda = 0.0;
//	double ro = 1.0;
//	double lambdaIncCoef = 2.0;
//	// divide number and new epsilon matrix
//	CvMat *newEpsMat = cvCreateMat(leftPoints.size(), 1, CV_64FC1);
//	CvMat *newDivide = cvCreateMat(1, 1, CV_64FC1);
//
//	// calculate JtEps
//	cvTranspose(JMatEM, JtMat);
//	// calculate JtJ matrix
//	cvMulTransposed(JMatEM, JtJMat, 1);
//	// calculate epsMat
//	cvCopy(errMatEM, epsMat);
//	// calculate JtMat
//	cvMatMul(JtMat, epsMat, JtEpsMat);
//	// set lambda
//	lambda = pow(10,8) * getMaxDiagonalElement(JtJMat);
//    
//    // check if infinity norm of JtEps equal or smaller than err
//    // if statement is true, break loop
//    if(getInfinityNorm(JtEpsMat) <= err) {
//        // release matrices
//        cvReleaseMat(&JtMat);
//        cvReleaseMat(&JtJMat);
//        cvReleaseMat(&JtEpsMat);
//        cvReleaseMat(&epsMat);
//        cvReleaseMat(&currentMatE);
//        cvReleaseMat(&deltaMat);
//        cvReleaseMat(&deltaMatT);
//        cvReleaseMat(&lambdaDeltaMat);
//        cvReleaseMat(&newEpsMat);
//        cvReleaseMat(&newDivide);
//
//        return matE;
//    }
//    
//	// go on with decreasing iteration count
//	while(--iters) {
//		// calculate Delta from (JtJ + lambda . I) . delta = JtEps
//		for(int i=0 ; i < JtJMat->rows ; ++i) {
//			CV_MAT_ELEM(*JtJMat, double, i, i) += lambda;
//		}
//
//		// solve (JtJ + lambda . I) . Delta = JtEps
//		cvSolve(JtJMat, JtEpsMat, deltaMat);
//		
//		// compare norms of deltaMat and MatE
//		if(cvNorm(deltaMat) <= LM_THRESHOLD * cvNorm(matE)) {		
//			// release matrices
//			cvReleaseMat(&JtMat);
//			cvReleaseMat(&JtJMat);
//			cvReleaseMat(&JtEpsMat);
//			cvReleaseMat(&epsMat);
//			cvReleaseMat(&currentMatE);
//			cvReleaseMat(&deltaMat);
//			cvReleaseMat(&deltaMatT);
//			cvReleaseMat(&lambdaDeltaMat);
//			cvReleaseMat(&newEpsMat);
//			cvReleaseMat(&newDivide);
//
//			return matE;
//		}
//		else {
//			// add deltaMat to prevMatF and find new matE
//			cvAdd(matE, deltaMat, currentMatE);
//			// calculate new error
//			cvCopy(getEpsMatrixForEM(leftPoints, rightPoints, convert9x1to3x3(currentMatE)), newEpsMat);
//			// find transpose of deltaMat
//			cvTranspose(deltaMat, deltaMatT);
//
//			// find lambda * deltaMat
//			for(int i = 0 ; i < lambdaDeltaMat->rows ; ++i) {
//				for(int j = 0 ; j < lambdaDeltaMat->cols ; ++j) {
//					CV_MAT_ELEM(*lambdaDeltaMat, double, i, j) = CV_MAT_ELEM(*deltaMat, double, i, j) * lambda;
//				}
//			}
//
//			// find lambdaDeltaMat + JtEpsMat
//			cvAdd(lambdaDeltaMat, JtEpsMat, lambdaDeltaMat);
//			cvMatMul(deltaMatT, lambdaDeltaMat, newDivide);
//			
//			// set ro if newDivide is not infinite
//			if(CV_MAT_ELEM(*newDivide, double, 0, 0) <= DBL_MAX && CV_MAT_ELEM(*newDivide, double, 0, 0) >= -DBL_MAX)
//				ro = (pow(cvNorm(epsMat), 2) - pow(cvNorm(newEpsMat), 2)) / CV_MAT_ELEM(*newDivide, double, 0, 0);
//			else
//				ro = 0.0;
//
//			// update some values
//			if(ro > 0) {
//				// copy currentMatE to matE
//				cvCopy(currentMatE, matE);
//				// calculate new epsMat and JtEpsMat
//				cvCopy(newEpsMat, epsMat);
//				// update Jacobian matrix
//				createJacobianMatrixForEM(leftPoints, rightPoints, convert9x1to3x3(matE));
//				// calculate JtEps
//				cvTranspose(JMatEM, JtMat);
//				// calculate JtJ matrix
//				cvMulTransposed(JMatEM, JtJMat, 1);
//				// calculate epsMat
//				cvCopy(errMatEM, epsMat);
//				// calculate JtEpsMat
//				cvMatMul(JtMat, epsMat, JtEpsMat);
//
//				// check if infinity norm of JtEps equal or smaller than err
//				// if statement is true, break loop
//				if(getInfinityNorm(JtEpsMat) <= err) {
//					// release matrices
//					cvReleaseMat(&JtMat);
//					cvReleaseMat(&JtJMat);
//					cvReleaseMat(&JtEpsMat);
//					cvReleaseMat(&epsMat);
//					cvReleaseMat(&currentMatE);
//					cvReleaseMat(&deltaMat);
//					cvReleaseMat(&deltaMatT);
//					cvReleaseMat(&lambdaDeltaMat);
//					cvReleaseMat(&newEpsMat);
//					cvReleaseMat(&newDivide);
//
//					return matE;
//				}
//
//				// update lambda 
//				lambda *= MAX(1/3, 1 - (double)pow((2*ro - 1), 3));
//				lambdaIncCoef = 2;
//			}
//			else {
//				lambda *= lambdaIncCoef;
//				lambdaIncCoef *= 2;
//
//				// if lambda becomes overflow, break loop and exit from function
//				if(lambda >= DBL_MAX && lambda <= -DBL_MAX) {
//					// release matrices
//					cvReleaseMat(&JtMat);
//					cvReleaseMat(&JtJMat);
//					cvReleaseMat(&JtEpsMat);
//					cvReleaseMat(&epsMat);
//					cvReleaseMat(&currentMatE);
//					cvReleaseMat(&deltaMat);
//					cvReleaseMat(&deltaMatT);
//					cvReleaseMat(&lambdaDeltaMat);
//					cvReleaseMat(&newEpsMat);
//					cvReleaseMat(&newDivide);
//
//					return matE;
//				}
//			}
//		} // end of else
//	} // end of while(--iters)
//
//	// release matrices
//	cvReleaseMat(&JtMat);
//	cvReleaseMat(&JtJMat);
//	cvReleaseMat(&JtEpsMat);
//	cvReleaseMat(&epsMat);
//	cvReleaseMat(&currentMatE);
//	cvReleaseMat(&deltaMat);
//	cvReleaseMat(&deltaMatT);
//	cvReleaseMat(&lambdaDeltaMat);
//	cvReleaseMat(&newEpsMat);
//	cvReleaseMat(&newDivide);
//
//	return matE;
//}

//// function that creates Jacobian matrix for essential matrix
//void ABLevenbergMarquardt::createJacobianMatrixForEM(const vector<CvPoint3D64f> leftPoints, const vector<CvPoint3D64f> rightPoints, const CvMat *essMat)
//{
//	/************************************************************/
//	/*     E.x = | A B C |T     |	ET.x' = | A' B' C' |T		*/
//	/*     A = f0x + f1y + f3z  |	A' = f0x' + f3y' + f6z'		*/
//	/*     B = f3x + f4y + f5z	|	B' = f1x' + f4y' + f7z'		*/
//	/*     C = f6x + f7y + f8z	|	C' = f2x' + f5y' + f8z'		*/
//	/************************************************************/
//	/*     x'T.E.x = D = (f0xx' + f1yx' + f2x'z) +				*/
//	/*				     (f3xy' + f4yy' + f5y'z) +				*/
//	/*				     (f6xz' + f7yz' + f8z'z)				*/
//	/*															*/
//	/*            M = A2 + B2 + A'2 + B'2						*/
//	/************************************************************/
//	/* Sampson Error: g = (x'T . E . x) / A2 + B2 + A'2 + B'2   */
//	/************************************************************/
//	/*		 J = dg = (x'T . E . x)' / (A2 + B2 + A'2 + B'2)'   */
//	/************************************************************/
//
//	// specify row and column sizes of Jacobian Matrix
//	const int row = (int)leftPoints.size();
//	const int col = (int)essMat->rows * (int)essMat->cols;
//	register double D;
//	register double M;
//	register double A, Aprime, B, Bprime;
//	// derivative variables these are derivation of g by f values
//	register double gf0, gf1, gf2, gf3, gf4, gf5, gf6, gf7, gf8;
//	// allocate memory for Jacobian and error matrices
//	JMatEM = cvCreateMat(row, col, CV_64FC1);
//	errMatEM = cvCreateMat(row, 1, CV_64FC1);
//
//	// fill Jacobian matrix
//	for(int i=0 ; i < row ; ++i) {
//		// set A,B, Aprime and Bprime
//		A = CV_MAT_ELEM(*essMat, double, 0, 0) * leftPoints.at(i).x + CV_MAT_ELEM(*essMat, double, 0, 1) * leftPoints.at(i).y + CV_MAT_ELEM(*essMat, double, 0, 2);
//		B = CV_MAT_ELEM(*essMat, double, 1, 0) * leftPoints.at(i).x + CV_MAT_ELEM(*essMat, double, 1, 1) * leftPoints.at(i).y + CV_MAT_ELEM(*essMat, double, 1, 2);
//		Aprime = CV_MAT_ELEM(*essMat, double, 0, 0) * rightPoints.at(i).x + CV_MAT_ELEM(*essMat, double, 1, 0) * rightPoints.at(i).y + CV_MAT_ELEM(*essMat, double, 2, 0);
//		Bprime = CV_MAT_ELEM(*essMat, double, 0, 1) * rightPoints.at(i).x + CV_MAT_ELEM(*essMat, double, 1, 1) * rightPoints.at(i).y + CV_MAT_ELEM(*essMat, double, 2, 1);
//		// set D
//		D = CV_MAT_ELEM(*essMat, double, 0, 0) * leftPoints.at(i).x * rightPoints.at(i).x + 
//			CV_MAT_ELEM(*essMat, double, 0, 1) * leftPoints.at(i).y * rightPoints.at(i).x + 
//			CV_MAT_ELEM(*essMat, double, 0, 2) * rightPoints.at(i).x * leftPoints.at(i).z + 
//			CV_MAT_ELEM(*essMat, double, 1, 0) * leftPoints.at(i).x * rightPoints.at(i).y + 
//			CV_MAT_ELEM(*essMat, double, 1, 1) * leftPoints.at(i).y * rightPoints.at(i).y + 
//			CV_MAT_ELEM(*essMat, double, 1, 2) * rightPoints.at(i).y * leftPoints.at(i).z + 
//			CV_MAT_ELEM(*essMat, double, 2, 0) * leftPoints.at(i).x * rightPoints.at(i).z + 
//			CV_MAT_ELEM(*essMat, double, 2, 1) * leftPoints.at(i).y * rightPoints.at(i).z +
//			CV_MAT_ELEM(*essMat, double, 2, 2) * leftPoints.at(i).z * rightPoints.at(i).z ;
//
//		// set M
//		M = pow(A, 2) + pow(B, 2) + pow(Aprime, 2) + pow(Bprime, 2);
//		// set gfX variables
//		gf0 = (leftPoints.at(i).x * rightPoints.at(i).x / M) + ((2 * D * (A * leftPoints.at(i).x + Aprime * rightPoints.at(i).x)) / pow(M, 2));
//		gf1 = (leftPoints.at(i).y * rightPoints.at(i).x / M) + ((2 * D * (A * leftPoints.at(i).y + Bprime * rightPoints.at(i).x)) / pow(M, 2));
//		gf2 = (rightPoints.at(i).x / M) + ((2 * D * (A * leftPoints.at(i).z)) / pow(M, 2));
//		gf3 = (leftPoints.at(i).x * rightPoints.at(i).y / M) + ((2 * D * (B * leftPoints.at(i).x + Aprime * rightPoints.at(i).y)) / pow(M, 2));
//		gf4 = (leftPoints.at(i).y * rightPoints.at(i).y / M) + ((2 * D * (B * leftPoints.at(i).y + Bprime * rightPoints.at(i).y)) / pow(M, 2));
//		gf5 = (rightPoints.at(i).y / M) + ((2 * D * (B * leftPoints.at(i).z)) / pow(M, 2));
//		gf6 = (leftPoints.at(i).x / M) + ((2 * D * (Aprime * rightPoints.at(i).z)) / pow(M, 2));
//		gf7 = (leftPoints.at(i).y / M) + ((2 * D * (Bprime * rightPoints.at(i).z)) / pow(M, 2));
//		gf8 = 1 / M; // + ((2 * D * (0)) / pow(M, 2));
//
//		// fill a row of Jacobian matrix
//		CV_MAT_ELEM(*JMatEM, double, i, 0) = gf0; 
//		CV_MAT_ELEM(*JMatEM, double, i, 1) = gf1;
//		CV_MAT_ELEM(*JMatEM, double, i, 2) = gf2;
//		CV_MAT_ELEM(*JMatEM, double, i, 3) = gf3;
//		CV_MAT_ELEM(*JMatEM, double, i, 4) = gf4;
//		CV_MAT_ELEM(*JMatEM, double, i, 5) = gf5;
//		CV_MAT_ELEM(*JMatEM, double, i, 6) = gf6;
//		CV_MAT_ELEM(*JMatEM, double, i, 7) = gf7;
//		CV_MAT_ELEM(*JMatEM, double, i, 8) = gf8;
//	} // end of for
//
//	// set error matrix
//	cvCopy(getEpsMatrixForEM(leftPoints, rightPoints, essMat), errMatEM);
//
//	return;
//}

//// function that gets new epsilon matrix for essential matrix
//CvMat *ABLevenbergMarquardt::getEpsMatrixForEM(const vector<CvPoint3D64f> leftPoints, const vector<CvPoint3D64f> rightPoints, const CvMat *E) {
//	CvMat *newErr = cvCreateMat((int)leftPoints.size(), 1, CV_64FC1);
//	CvMat *X1  = cvCreateMat(3, 1, CV_64FC1);
//	CvMat *X2t = cvCreateMat(1, 3, CV_64FC1);
//	CvMat *X2tEX1 = cvCreateMat(1, 1, CV_64FC1);
//
//	// set error matrix
//	for(int i=0 ; i < (int)leftPoints.size() ; ++i) {
//		// create X2 point
//		CV_MAT_ELEM(*X2t, double, 0, 0) = rightPoints.at(i).x;
//		CV_MAT_ELEM(*X2t, double, 0, 1) = rightPoints.at(i).y;
//		CV_MAT_ELEM(*X2t, double, 0, 2) = rightPoints.at(i).z;
//		// create X1 point
//		CV_MAT_ELEM(*X1, double, 0, 0) = leftPoints.at(i).x;
//		CV_MAT_ELEM(*X1, double, 1, 0) = leftPoints.at(i).y;
//		CV_MAT_ELEM(*X1, double, 2, 0) = leftPoints.at(i).z;
//		// find result of X2t . F . X1
//		cvMatMul(X2t, E, X2t);
//		cvMatMul(X2t, X1, X2tEX1);
//
//		CV_MAT_ELEM(*newErr, double, i, 0) = 0 - CV_MAT_ELEM(*X2tEX1, double, 0, 0);
//	}
//
//	// release matrices
//	cvReleaseMat(&X1);
//	cvReleaseMat(&X2t);
//	cvReleaseMat(&X2tEX1);
//
//	return newErr;
//}