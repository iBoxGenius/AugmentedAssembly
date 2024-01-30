#include "AssemblyPart.h"

AssemblyPart::AssemblyPart()
{
}

AssemblyPart::~AssemblyPart()
{
}

void AssemblyPart::SetDescriptor(cv::Mat& desc)	//check if x*128
{
	m_descriptor = desc;
}

cv::Mat AssemblyPart::GetDescriptor()
{
	return m_descriptor;
}
