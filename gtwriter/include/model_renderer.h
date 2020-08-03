//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef GT_WRITER_MODEL_RENDERER_H
#define GT_WRITER_MODEL_RENDERER_H

#include "model.h"
#include <iostream>
#include "renderer.h"
#include "configuration.h"

class ModelRenderer {


public:
	ModelRenderer(const Model &model, const Configuration &configuration, bool scale_focal_length = false);
	void ModelRenderer::render(Eigen::Matrix4f &world_to_cam, cv::Mat &depth, cv::Mat &color);

private:
	std::shared_ptr<RendererInterface> renderer;

};

#endif
