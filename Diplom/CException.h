#pragma once
#include <exception>
#include <string>
#include <map>
class CException : public std::exception
{
public:
	CException(const std::string& message, const std::map<const std::string, double>& args) noexcept;
	virtual const char* what() const override;
	double getValueByKey(const std::string& key) const;
private:
	std::string message;
	std::map<const std::string, double> args;
};

