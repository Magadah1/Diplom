#include "CException.h"

CException::CException(const std::string& message, const std::map<const std::string, double>& args) noexcept
{
	this->message = message;
	for (std::map<std::string, double>::const_iterator it = args.begin(); it != args.end(); ++it)
	{
		std::string temp = "\n" + it->first + " = " + std::to_string(it->second);
		this->message += temp;
	}
	this->message.append("\n");

	this->args = args;
}

const char* CException::what() const
{
	return this->message.c_str();
}

double CException::getValueByKey(const std::string& key) const
{
	return this->args.at(key);
}
