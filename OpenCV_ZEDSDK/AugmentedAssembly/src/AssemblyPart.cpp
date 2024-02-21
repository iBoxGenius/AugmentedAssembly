#include "AssemblyPart.h"

AssemblyPart::AssemblyPart()
{
}

AssemblyPart::~AssemblyPart()
{
}

void AssemblyPart::SetDescriptor(cv::Mat& desc)	
{
	m_descriptor = desc;
}

cv::Mat AssemblyPart::GetDescriptor()
{
	return m_descriptor;
}
